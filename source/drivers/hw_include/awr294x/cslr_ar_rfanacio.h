/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_ar_rfanacio.h
*/
#ifndef CSLR_AR_RFANACIO_H_
#define CSLR_AR_RFANACIO_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_4[4];
    volatile uint32_t RX1_I_IFA_CTRL_1;
    volatile uint32_t RX1_I_IFA_REFGEN_LDO_CTRL;
    volatile uint8_t  Resv_16[4];
    volatile uint32_t RX1_I_IFA_CTRL_2;
    volatile uint32_t RX1_I_IFA_CTRL_3;
    volatile uint32_t RX1_I_IFA_CTRL_4;
    volatile uint32_t RX1_I_IFA_CTRL_5;
    volatile uint32_t RX1_I_IFA_TEST_CTRL;
    volatile uint32_t RX1_ADC_I_ANA_LDO_CTRL;
    volatile uint32_t RX1_ADC_I_CTRL_1;
    volatile uint32_t RX1_ADC_I_CTRL_2;
    volatile uint32_t RX1_ADC_I_CTRL_3;
    volatile uint32_t RX1_ADC_I_TMUX_CTRL;
    volatile uint32_t RX1_ADC_I_DIG_LDO_CTRL;
    volatile uint32_t RX1_CTRL;
    volatile uint8_t  Resv_68[4];
    volatile uint32_t RX1_FE_TMUX_SPARE_CTRL;
    volatile uint8_t  Resv_84[12];
    volatile uint32_t RX1_RXFE_CTRL_2;
    volatile uint32_t RX1_RXFE_CTRL_1;
    volatile uint8_t  Resv_152[60];
    volatile uint32_t LODIST_CTRL3;
    volatile uint32_t LODIST_ATEST_CTRL3;
    volatile uint8_t  Resv_216[56];
    volatile uint32_t RX2_RXFE_CTRL_1;
    volatile uint32_t RX2_RXFE_CTRL_2;
    volatile uint8_t  Resv_236[12];
    volatile uint32_t RX2_FE_TMUX_SPARE_CTRL;
    volatile uint8_t  Resv_244[4];
    volatile uint32_t RX2_CTRL;
    volatile uint32_t RX2_ADC_I_DIG_LDO_CTRL;
    volatile uint32_t RX2_ADC_I_TMUX_CTRL;
    volatile uint32_t RX2_ADC_I_CTRL_3;
    volatile uint32_t RX2_ADC_I_CTRL_2;
    volatile uint32_t RX2_ADC_I_CTRL_1;
    volatile uint32_t RX2_ADC_I_ANA_LDO_CTRL;
    volatile uint32_t RX2_I_IFA_TEST_CTRL;
    volatile uint32_t RX2_I_IFA_CTRL_5;
    volatile uint32_t RX2_I_IFA_CTRL_4;
    volatile uint32_t RX2_I_IFA_CTRL_3;
    volatile uint32_t RX2_I_IFA_CTRL_2;
    volatile uint8_t  Resv_296[4];
    volatile uint32_t RX2_I_IFA_REFGEN_LDO_CTRL;
    volatile uint32_t RX2_I_IFA_CTRL_1;
    volatile uint32_t RX_TOPMUX_BUF_CTRL;
    volatile uint32_t RX_TOP_ADC_PWRUP_BYPASS_CTRL;
    volatile uint32_t RX_TOP_ADC_PWRUP_BYPASS_EN;
    volatile uint32_t RX_TOP_SPARE_REG1;
    volatile uint32_t RX_TOP_SPARE_REG2;
    volatile uint32_t RX_REFSYS_TMUX_SPARE_CTRL;
    volatile uint32_t RX3_I_IFA_CTRL_1;
    volatile uint32_t RX3_I_IFA_REFGEN_LDO_CTRL;
    volatile uint8_t  Resv_340[4];
    volatile uint32_t RX3_I_IFA_CTRL_2;
    volatile uint32_t RX3_I_IFA_CTRL_3;
    volatile uint32_t RX3_I_IFA_CTRL_4;
    volatile uint32_t RX3_I_IFA_CTRL_5;
    volatile uint32_t RX3_I_IFA_TEST_CTRL;
    volatile uint32_t RX3_ADC_I_ANA_LDO_CTRL;
    volatile uint32_t RX3_ADC_I_CTRL_1;
    volatile uint32_t RX3_ADC_I_CTRL_2;
    volatile uint32_t RX3_ADC_I_CTRL_3;
    volatile uint32_t RX3_ADC_I_TMUX_CTRL;
    volatile uint32_t RX3_ADC_I_DIG_LDO_CTRL;
    volatile uint32_t RX3_CTRL;
    volatile uint8_t  Resv_392[4];
    volatile uint32_t RX3_FE_TMUX_SPARE_CTRL;
    volatile uint8_t  Resv_408[12];
    volatile uint32_t RX3_RXFE_CTRL_2;
    volatile uint32_t RX3_RXFE_CTRL_1;
    volatile uint8_t  Resv_472[56];
    volatile uint32_t LODIST_ATEST_CTRL4;
    volatile uint32_t LODIST_CTRL4;
    volatile uint8_t  Resv_540[60];
    volatile uint32_t RX4_RXFE_CTRL_1;
    volatile uint32_t RX4_RXFE_CTRL_2;
    volatile uint8_t  Resv_560[12];
    volatile uint32_t RX4_FE_TMUX_SPARE_CTRL;
    volatile uint8_t  Resv_568[4];
    volatile uint32_t RX4_CTRL;
    volatile uint32_t RX_TEST_EN;
    volatile uint32_t RX4_ADC_I_DIG_LDO_CTRL;
    volatile uint32_t RX4_ADC_I_TMUX_CTRL;
    volatile uint32_t RX4_ADC_I_CTRL_3;
    volatile uint32_t RX4_ADC_I_CTRL_2;
    volatile uint32_t RX4_ADC_I_CTRL_1;
    volatile uint32_t RX4_ADC_I_ANA_LDO_CTRL;
    volatile uint32_t RX4_I_IFA_TEST_CTRL;
    volatile uint32_t RX4_I_IFA_CTRL_5;
    volatile uint32_t RX4_I_IFA_CTRL_4;
    volatile uint32_t RX4_I_IFA_CTRL_3;
    volatile uint32_t RX4_I_IFA_CTRL_2;
    volatile uint8_t  Resv_624[4];
    volatile uint32_t RX4_I_IFA_REFGEN_LDO_CTRL;
    volatile uint32_t RX4_I_IFA_CTRL_1;
    volatile uint32_t TX_TOP_BIST_LNA_CTRL;
    volatile uint8_t  Resv_640[4];
    volatile uint32_t TX_TOP_BIST_EN_BYPASS;
    volatile uint32_t TX_TOP_EN;
    volatile uint32_t TX_TOP_TESTMUX_CTRL;
    volatile uint8_t  Resv_656[4];
    volatile uint32_t TX_CONFIG_LOOPBACK;
    volatile uint32_t TX1_DAC_CTRL;
    volatile uint32_t TX1_DAC_CONFIG;
    volatile uint32_t TX1_PA_BIASCTRL_REG1;
    volatile uint32_t TX1_PA_BIASCTRL_REG2;
    volatile uint32_t TX1_PA_EN;
    volatile uint32_t TX1_PA_SPARES;
    volatile uint32_t TX1_PA_TESTMUX_CTRL;
    volatile uint8_t  Resv_692[4];
    volatile uint32_t TX1_DAC_DIGLDO_CTRL;
    volatile uint32_t TX1_DAC_ANALDO_CTRL;
    volatile uint8_t  Resv_704[4];
    volatile uint32_t TX1_IQGEN_BIAS_CTRL;
    volatile uint32_t TX1_IQGENLDO_CTRL;
    volatile uint8_t TX1_CTRL_RC_FILT;
    volatile uint8_t  Resv_716[3];
    volatile uint16_t TX1_PS_TMUXCTRL;
    volatile uint8_t  Resv_720[2];
    volatile uint32_t TX1_PS_BIASCTRL_REG1;
    volatile uint16_t TX1_PS_SPARES;
    volatile uint8_t  Resv_728[2];
    volatile uint32_t TX1_BIST_LNA_CTRL;
    volatile uint8_t  Resv_744[12];
    volatile uint16_t TX_PS_MODE_CTRL;
    volatile uint8_t  Resv_752[6];
    volatile uint32_t TX_PS_EN;
    volatile uint8_t  Resv_760[4];
    volatile uint32_t TX2_BIST_LNA_CTRL;
    volatile uint8_t  Resv_768[4];
    volatile uint32_t TX2_PS_BIASCTRL_REG1;
    volatile uint16_t TX2_PS_TMUXCTRL;
    volatile uint8_t  Resv_776[2];
    volatile uint8_t TX2_CTRL_RC_FILT;
    volatile uint8_t  Resv_780[3];
    volatile uint32_t TX2_IQGENLDO_CTRL;
    volatile uint32_t TX2_IQGEN_BIAS_CTRL;
    volatile uint8_t  Resv_792[4];
    volatile uint32_t TX2_DAC_ANALDO_CTRL;
    volatile uint32_t TX2_DAC_DIGLDO_CTRL;
    volatile uint8_t  Resv_804[4];
    volatile uint32_t TX2_PA_TESTMUX_CTRL;
    volatile uint32_t TX2_PA_SPARES;
    volatile uint32_t TX2_PA_EN;
    volatile uint32_t TX2_PA_BIASCTRL_REG2;
    volatile uint32_t TX2_PA_BIASCTRL_REG1;
    volatile uint32_t TX2_DAC_CONFIG;
    volatile uint32_t TX2_DAC_CTRL;
    volatile uint8_t  Resv_836[4];
    volatile uint32_t TX3_BIST_LNA_CTRL;
    volatile uint8_t  Resv_844[4];
    volatile uint32_t TX3_PS_BIASCTRL_REG1;
    volatile uint16_t TX3_PS_TMUXCTRL;
    volatile uint8_t  Resv_852[2];
    volatile uint8_t TX3_CTRL_RC_FILT;
    volatile uint8_t  Resv_856[3];
    volatile uint32_t TX3_IQGENLDO_CTRL;
    volatile uint32_t TX3_IQGEN_BIAS_CTRL;
    volatile uint8_t  Resv_868[4];
    volatile uint32_t TX3_DAC_ANALDO_CTRL;
    volatile uint32_t TX3_DAC_DIGLDO_CTRL;
    volatile uint8_t  Resv_880[4];
    volatile uint32_t TX3_PA_TESTMUX_CTRL;
    volatile uint32_t TX3_PA_SPARES;
    volatile uint32_t TX3_PA_EN;
    volatile uint32_t TX3_PA_BIASCTRL_REG2;
    volatile uint32_t TX3_PA_BIASCTRL_REG1;
    volatile uint32_t TX3_DAC_CONFIG;
    volatile uint32_t TX3_DAC_CTRL;
    volatile uint8_t  Resv_912[4];
    volatile uint32_t LODIST_CTRL5;
    volatile uint32_t LODIST_ATEST_CTRL5;
    volatile uint8_t  Resv_924[4];
    volatile uint32_t LODIST_CTRL6;
    volatile uint32_t LODIST_ATEST_CTRL6;
    volatile uint8_t  Resv_936[4];
    volatile uint32_t LODIST_CTRL2;
    volatile uint32_t LODIST_ATEST_CTRL2;
    volatile uint8_t  Resv_948[4];
    volatile uint32_t REFSYS_CTRL_REG;
    volatile uint8_t  Resv_964[12];
    volatile uint32_t TW_CTRL_REG;
    volatile uint8_t  Resv_972[4];
    volatile uint32_t TW_ANA_TMUX_CTRL;
    volatile uint32_t RFANA_TOP_CTRL1;
    volatile uint8_t  Resv_1008[28];
    volatile uint32_t LODIST_LDO_CTRL1;
    volatile uint32_t LODIST_CTRL1;
    volatile uint32_t LODIST_ATEST_CTRL1;
    volatile uint32_t LODIST_GEN_SPARES;
    volatile uint32_t CLK_CTRL_REG1_SYNC_20G;
    volatile uint32_t CLK_CTRL_REG2_SYNC_20G;
    volatile uint32_t CLK_CTRL_REG3_SYNC_20G;
    volatile uint32_t CLK_CTRL_REG2_CLKTOP;
    volatile uint32_t CLK_CTRL_REG8_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG6_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG6_SYNTH;
    volatile uint32_t CLK_CTRL_REG4_SYNTH;
    volatile uint32_t CLK_CTRL_REG5_SYNTH;
    volatile uint32_t CLK_CTRL_REG4_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG5_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG7_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG1_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG1_SYNTH;
    volatile uint32_t CLK_CTRL_REG2_SYNTH;
    volatile uint32_t CLK_CTRL_REG3_SYNTH;
    volatile uint32_t CLK_CTRL_REG1_APLL;
    volatile uint32_t CLK_CTRL_REG2_APLL;
    volatile uint32_t CLK_CTRL_REG3_APLL;
    volatile uint32_t CLK_CTRL_REG4_APLL;
    volatile uint32_t CLK_CTRL_REG1_CLKTOP;
    volatile uint32_t CLK_CTRL_REG3_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG2_LDO_CLKTOP;
    volatile uint32_t CLK_CTRL_REG1_XO_SLICER;
    volatile uint32_t EFUSE_SPARE_REG;
    volatile uint32_t ODP_BIST_CTRL;
    volatile uint32_t REFSYS_SPARE_REG2;
    volatile uint32_t REFSYS_SPARE_REG3;
    volatile uint32_t RX_TOP_SPARE_REG3;
    volatile uint32_t RX_TOP_SPARE_REG4;
    volatile uint32_t TX_PWRSW_CTRL;
    volatile uint32_t TX4_BIST_LNA_CTRL;
    volatile uint8_t TX4_CTRL_RC_FILT;
    volatile uint8_t  Resv_1156[3];
    volatile uint32_t TX4_DAC_ANALDO_CTRL;
    volatile uint32_t ANA_DFE_ISO_CTRL;
    volatile uint32_t TX4_DAC_CONFIG;
    volatile uint32_t TX4_DAC_CTRL;
    volatile uint32_t TX4_DAC_DIGLDO_CTRL;
    volatile uint32_t TX4_IQGEN_BIAS_CTRL;
    volatile uint32_t TX4_IQGENLDO_CTRL;
    volatile uint32_t TX4_PA_BIASCTRL_REG1;
    volatile uint32_t TX4_PA_BIASCTRL_REG2;
    volatile uint32_t TX4_PA_EN;
    volatile uint32_t TX4_PA_SPARES;
    volatile uint32_t TX4_PA_TESTMUX_CTRL;
    volatile uint32_t TX4_PS_BIASCTRL_REG1;
    volatile uint16_t TX4_PS_TMUXCTRL;
    volatile uint8_t  Resv_1272[62];
    volatile uint32_t TXPALOOPBCKCLK;
    volatile uint32_t RXIFALOOPBCKCLK;
    volatile uint32_t RX_STATUS_REG1;
    volatile uint32_t RX_STATUS_REG2;
    volatile uint32_t LODIST_STATUS_REG;
    volatile uint32_t TX_STATUS_REG;
    volatile uint32_t WU_MODE_REG;
    volatile uint32_t WU_STATUS_REG;
    volatile uint32_t WU_SPARE_OUT;
    volatile uint32_t CLK_STATUS_REG;
} CSL_ar_rfanacioRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1                                       (0x00000004U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL                              (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2                                       (0x00000010U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3                                       (0x00000014U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4                                       (0x00000018U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5                                       (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL                                    (0x00000020U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL                                 (0x00000024U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1                                       (0x00000028U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2                                       (0x0000002CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3                                       (0x00000030U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL                                    (0x00000034U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL                                 (0x00000038U)
#define CSL_AR_RFANACIO_RX1_CTRL                                               (0x0000003CU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL                                 (0x00000044U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2                                        (0x00000054U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1                                        (0x00000058U)
#define CSL_AR_RFANACIO_LODIST_CTRL3                                           (0x00000098U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3                                     (0x0000009CU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1                                        (0x000000D8U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2                                        (0x000000DCU)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL                                 (0x000000ECU)
#define CSL_AR_RFANACIO_RX2_CTRL                                               (0x000000F4U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL                                 (0x000000F8U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL                                    (0x000000FCU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3                                       (0x00000100U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2                                       (0x00000104U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1                                       (0x00000108U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL                                 (0x0000010CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL                                    (0x00000110U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5                                       (0x00000114U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4                                       (0x00000118U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3                                       (0x0000011CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2                                       (0x00000120U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL                              (0x00000128U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1                                       (0x0000012CU)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL                                     (0x00000130U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL                           (0x00000134U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN                             (0x00000138U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1                                      (0x0000013CU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2                                      (0x00000140U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL                              (0x00000144U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1                                       (0x00000148U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL                              (0x0000014CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2                                       (0x00000154U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3                                       (0x00000158U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4                                       (0x0000015CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5                                       (0x00000160U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL                                    (0x00000164U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL                                 (0x00000168U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1                                       (0x0000016CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2                                       (0x00000170U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3                                       (0x00000174U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL                                    (0x00000178U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL                                 (0x0000017CU)
#define CSL_AR_RFANACIO_RX3_CTRL                                               (0x00000180U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL                                 (0x00000188U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2                                        (0x00000198U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1                                        (0x0000019CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4                                     (0x000001D8U)
#define CSL_AR_RFANACIO_LODIST_CTRL4                                           (0x000001DCU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1                                        (0x0000021CU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2                                        (0x00000220U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL                                 (0x00000230U)
#define CSL_AR_RFANACIO_RX4_CTRL                                               (0x00000238U)
#define CSL_AR_RFANACIO_RX_TEST_EN                                             (0x0000023CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL                                 (0x00000240U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL                                    (0x00000244U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3                                       (0x00000248U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2                                       (0x0000024CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1                                       (0x00000250U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL                                 (0x00000254U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL                                    (0x00000258U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5                                       (0x0000025CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4                                       (0x00000260U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3                                       (0x00000264U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2                                       (0x00000268U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL                              (0x00000270U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1                                       (0x00000274U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL                                   (0x00000278U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS                                  (0x00000280U)
#define CSL_AR_RFANACIO_TX_TOP_EN                                              (0x00000284U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL                                    (0x00000288U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK                                     (0x00000290U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL                                           (0x00000294U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG                                         (0x00000298U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1                                   (0x0000029CU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2                                   (0x000002A0U)
#define CSL_AR_RFANACIO_TX1_PA_EN                                              (0x000002A4U)
#define CSL_AR_RFANACIO_TX1_PA_SPARES                                          (0x000002A8U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL                                    (0x000002ACU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL                                    (0x000002B4U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL                                    (0x000002B8U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL                                    (0x000002C0U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL                                      (0x000002C4U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT                                       (0x000002C8U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL                                        (0x000002CCU)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1                                   (0x000002D0U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES                                          (0x000002D4U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL                                      (0x000002D8U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL                                        (0x000002E8U)
#define CSL_AR_RFANACIO_TX_PS_EN                                               (0x000002F0U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL                                      (0x000002F8U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1                                   (0x00000300U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL                                        (0x00000304U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT                                       (0x00000308U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL                                      (0x0000030CU)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL                                    (0x00000310U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL                                    (0x00000318U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL                                    (0x0000031CU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL                                    (0x00000324U)
#define CSL_AR_RFANACIO_TX2_PA_SPARES                                          (0x00000328U)
#define CSL_AR_RFANACIO_TX2_PA_EN                                              (0x0000032CU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2                                   (0x00000330U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1                                   (0x00000334U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG                                         (0x00000338U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL                                           (0x0000033CU)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL                                      (0x00000344U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1                                   (0x0000034CU)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL                                        (0x00000350U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT                                       (0x00000354U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL                                      (0x00000358U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL                                    (0x0000035CU)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL                                    (0x00000364U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL                                    (0x00000368U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL                                    (0x00000370U)
#define CSL_AR_RFANACIO_TX3_PA_SPARES                                          (0x00000374U)
#define CSL_AR_RFANACIO_TX3_PA_EN                                              (0x00000378U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2                                   (0x0000037CU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1                                   (0x00000380U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG                                         (0x00000384U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL                                           (0x00000388U)
#define CSL_AR_RFANACIO_LODIST_CTRL5                                           (0x00000390U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5                                     (0x00000394U)
#define CSL_AR_RFANACIO_LODIST_CTRL6                                           (0x0000039CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6                                     (0x000003A0U)
#define CSL_AR_RFANACIO_LODIST_CTRL2                                           (0x000003A8U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2                                     (0x000003ACU)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG                                        (0x000003B4U)
#define CSL_AR_RFANACIO_TW_CTRL_REG                                            (0x000003C4U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL                                       (0x000003CCU)
#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1                                        (0x000003D0U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1                                       (0x000003F0U)
#define CSL_AR_RFANACIO_LODIST_CTRL1                                           (0x000003F4U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1                                     (0x000003F8U)
#define CSL_AR_RFANACIO_LODIST_GEN_SPARES                                      (0x000003FCU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G                                 (0x00000400U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G                                 (0x00000404U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G                                 (0x00000408U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP                                   (0x0000040CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP                               (0x00000410U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP                               (0x00000414U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH                                    (0x00000418U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH                                    (0x0000041CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH                                    (0x00000420U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP                               (0x00000424U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP                               (0x00000428U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP                               (0x0000042CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP                               (0x00000430U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH                                    (0x00000434U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH                                    (0x00000438U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH                                    (0x0000043CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL                                     (0x00000440U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL                                     (0x00000444U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL                                     (0x00000448U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL                                     (0x0000044CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP                                   (0x00000450U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP                               (0x00000454U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP                               (0x00000458U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER                                (0x0000045CU)
#define CSL_AR_RFANACIO_EFUSE_SPARE_REG                                        (0x00000460U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL                                          (0x00000464U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2                                      (0x00000468U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3                                      (0x0000046CU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3                                      (0x00000470U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4                                      (0x00000474U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL                                          (0x00000478U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL                                      (0x0000047CU)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT                                       (0x00000480U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL                                    (0x00000484U)
#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL                                       (0x00000488U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG                                         (0x0000048CU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL                                           (0x00000490U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL                                    (0x00000494U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL                                    (0x00000498U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL                                      (0x0000049CU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1                                   (0x000004A0U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2                                   (0x000004A4U)
#define CSL_AR_RFANACIO_TX4_PA_EN                                              (0x000004A8U)
#define CSL_AR_RFANACIO_TX4_PA_SPARES                                          (0x000004ACU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL                                    (0x000004B0U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1                                   (0x000004B4U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL                                        (0x000004B8U)
#define CSL_AR_RFANACIO_TXPALOOPBCKCLK                                         (0x000004F8U)
#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK                                        (0x000004FCU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1                                         (0x00000500U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2                                         (0x00000504U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG                                      (0x00000508U)
#define CSL_AR_RFANACIO_TX_STATUS_REG                                          (0x0000050CU)
#define CSL_AR_RFANACIO_WU_MODE_REG                                            (0x00000510U)
#define CSL_AR_RFANACIO_WU_STATUS_REG                                          (0x00000514U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT                                           (0x00000518U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG                                         (0x0000051CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RX1_I_IFA_CTRL_1 */

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MASK                  (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MASK          (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED0_MASK                        (0x00000020U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED0_SHIFT                       (0x00000005U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MASK        (0x000000C0U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_SHIFT       (0x00000006U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MASK                   (0x00001000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_EN_SHIFT                  (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MASK              (0x00002000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_SHIFT             (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MASK           (0x0001C000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED1_MASK                        (0x00020000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED1_SHIFT                       (0x00000011U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED1_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED1_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MASK         (0x000C0000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_SHIFT        (0x00000012U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MASK       (0x00300000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MASK                   (0x00400000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_EN_SHIFT                  (0x00000016U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MASK              (0x00800000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_SHIFT             (0x00000017U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MASK           (0x07000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_SHIFT          (0x00000018U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED2_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED2_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED2_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESERVED2_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MASK         (0x30000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_SHIFT        (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_1_RESETVAL                              (0x00000000U)

/* RX1_I_IFA_REFGEN_LDO_CTRL */

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MASK       (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_SHIFT      (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MASK      (0x00000020U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_SHIFT     (0x00000005U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MASK     (0x00000040U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_SHIFT    (0x00000006U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MASK       (0x00000080U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_SHIFT      (0x00000007U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MASK         (0x00000700U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MAX          (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MASK      (0x0000E000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_SHIFT     (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MAX       (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MASK           (0x00010000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_SHIFT          (0x00000010U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MASK (0x000E0000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MASK (0x00700000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_SHIFT (0x00000014U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MAX  (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MASK (0x03800000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_SHIFT (0x00000017U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MASK (0x1C000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_SHIFT (0x0000001AU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MASK (0xE0000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_SHIFT (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_REFGEN_LDO_CTRL_RESETVAL                     (0x00000000U)

/* RX1_I_IFA_CTRL_2 */

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_EN_MASK                    (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_EN_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MASK              (0x0000007EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_SHIFT             (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_EN_MASK                    (0x00000100U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_EN_SHIFT                   (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MASK                  (0x0000FC00U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_CTRL_SHIFT                 (0x0000000AU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_CTRL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_EN_MASK                   (0x00010000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_EN_SHIFT                  (0x00000010U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MASK           (0x00020000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_SHIFT          (0x00000011U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MASK                 (0x00040000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_SHIFT                (0x00000012U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_EN_MUX_SEL_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_EN_MUX_SEL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MASK       (0xFF800000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_SHIFT      (0x00000017U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MAX        (0x000001FFU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_2_RESETVAL                              (0x00000000U)

/* RX1_I_IFA_CTRL_3 */

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGA_RTRIM_MASK                    (0x0000007FU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGA_RTRIM_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGA_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGA_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGB_RTRIM_MASK                    (0x00003F80U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGB_RTRIM_SHIFT                   (0x00000007U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGB_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_RGB_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF_CTRL_MASK                     (0x003FC000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF_CTRL_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF_CTRL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF_CTRL_MAX                      (0x000000FFU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MASK               (0x00C00000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_SHIFT              (0x00000016U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MASK                 (0x06000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MAX                  (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_REDERVED0_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_REDERVED0_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_REDERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_REDERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MASK                 (0x10000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_SHIFT                (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_RESERVED0_MASK                        (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_RESERVED0_SHIFT                       (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_RESERVED0_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_3_RESETVAL                              (0x00000000U)

/* RX1_I_IFA_CTRL_4 */

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MASK                   (0x0000003FU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_CTRIM_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MASK                   (0x00000FC0U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_CTRIM_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_SEL_MASK                     (0x00007000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_SEL_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFA_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_SEL_MASK                     (0x00038000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_SEL_SHIFT                    (0x0000000FU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPFB_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_R1B_RTRIM_MASK                    (0x00FC0000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_R1B_RTRIM_SHIFT                   (0x00000012U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_R1B_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_R1B_RTRIM_MAX                     (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MASK          (0x07000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MASK           (0x18000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_SHIFT          (0x0000001BU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MASK       (0x20000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_SHIFT      (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_4_RESETVAL                              (0x00000000U)

/* RX1_I_IFA_CTRL_5 */

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_RSIG_RTRIM_MASK                   (0x000001FFU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_RSIG_RTRIM_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_RSIG_RTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_RSIG_RTRIM_MAX                    (0x000001FFU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_R1A_RTRIM_MASK                    (0x0003FE00U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_R1A_RTRIM_SHIFT                   (0x00000009U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_R1A_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_R1A_RTRIM_MAX                     (0x000001FFU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_LPF_CTRL_MASK                     (0x0FFC0000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_LPF_CTRL_SHIFT                    (0x00000012U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_LPF_CTRL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_LPF_CTRL_MAX                      (0x000003FFU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MASK           (0xF0000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_SHIFT          (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_CTRL_5_RESETVAL                              (0x00000000U)

/* RX1_I_IFA_TEST_CTRL */

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MASK        (0x00000002U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_SHIFT       (0x00000001U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MASK             (0x00000004U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_SHIFT            (0x00000002U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MASK          (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_SHIFT         (0x00000003U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_INPUT_TEST_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_INPUT_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED0_MASK                     (0x00000020U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED0_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED0_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MASK         (0x00000040U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_SHIFT        (0x00000006U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MASK         (0x00000080U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_SHIFT        (0x00000007U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED1_MASK                     (0x00000F00U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED1_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED1_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MASK    (0x00002000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_SHIFT   (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MASK      (0x00004000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_SHIFT     (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MASK    (0x00010000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MASK       (0x00040000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_SHIFT      (0x00000012U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MASK         (0x00100000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_SHIFT        (0x00000014U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MASK          (0x00800000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MASK (0x01000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED2_MASK                     (0xFE000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED2_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESERVED2_MAX                      (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_I_IFA_TEST_CTRL_RESETVAL                           (0x00000000U)

/* RX1_ADC_I_ANA_LDO_CTRL */

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_RESERVED1_MASK                  (0xFFFF0000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_RESERVED1_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_RESERVED1_MAX                   (0x0000FFFFU)

#define CSL_AR_RFANACIO_RX1_ADC_I_ANA_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX1_ADC_I_CTRL_1 */

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MASK               (0x00000007U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MASK               (0x00000038U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MASK               (0x000001C0U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_SHIFT              (0x00000006U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MASK                (0x00000E00U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_SHIFT               (0x00000009U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MASK             (0x00002000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MASK             (0x00008000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_SHIFT            (0x0000000FU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MASK           (0x00380000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_SHIFT          (0x00000013U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MASK              (0x00400000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_SHIFT             (0x00000016U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MASK          (0x03800000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MASK           (0x1C000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_SHIFT          (0x0000001AU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MASK              (0x20000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_SHIFT             (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MASK          (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MAX           (0x00000003U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_1_RESETVAL                              (0x00000000U)

/* RX1_ADC_I_CTRL_2 */

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MASK             (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MAX              (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MASK       (0x000000F0U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MASK       (0x00000F00U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_SHIFT      (0x00000008U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MASK       (0x0000F000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MASK       (0x000F0000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_SHIFT      (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MASK       (0x00F00000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MASK       (0x0F000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_SHIFT      (0x00000018U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MASK        (0xF0000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_2_RESETVAL                              (0x00000000U)

/* RX1_ADC_I_CTRL_3 */

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MASK        (0x0000000FU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_SHIFT       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MASK                 (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_SHIFT                (0x00000004U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MASK              (0x00000020U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_SHIFT             (0x00000005U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MASK              (0x000003C0U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MASK        (0x00000C00U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_SHIFT       (0x0000000AU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MASK           (0x00003000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MASK           (0x0000C000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MASK              (0x00080000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_SHIFT             (0x00000013U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MASK              (0x00100000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_SHIFT             (0x00000014U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MASK                 (0x00200000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_SHIFT                (0x00000015U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MASK             (0x00400000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_SHIFT            (0x00000016U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MASK           (0x00800000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_SHIFT          (0x00000017U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MASK                   (0x01000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_IFA_CAL_CLK_EN_SHIFT                  (0x00000018U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_IFA_CAL_CLK_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_EN_MASK                     (0x02000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_EN_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_IBIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MASK            (0x04000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_SHIFT           (0x0000001AU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MASK            (0x08000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_SHIFT           (0x0000001BU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MASK              (0x10000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_SHIFT             (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EN_MASK                    (0x20000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EN_SHIFT                   (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_TX_DAC_CLK_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MASK             (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_SHIFT            (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MAX              (0x00000003U)

#define CSL_AR_RFANACIO_RX1_ADC_I_CTRL_3_RESETVAL                              (0x00000000U)

/* RX1_ADC_I_TMUX_CTRL */

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MASK              (0x00000004U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_SHIFT             (0x00000002U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MASK             (0x00000008U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_SHIFT            (0x00000003U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MASK              (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MASK               (0x00000020U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_SHIFT              (0x00000005U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MASK                 (0x00000200U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_SHIFT                (0x00000009U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MASK             (0x00000400U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_SHIFT            (0x0000000AU)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_VSSA_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_VSSA_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_VSSA_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_VSSA_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MASK                  (0x00002000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_SHIFT                 (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_RESERVED0_MASK                     (0xFFFFC000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_RESERVED0_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_RESERVED0_MAX                      (0x0003FFFFU)

#define CSL_AR_RFANACIO_RX1_ADC_I_TMUX_CTRL_RESETVAL                           (0x00000000U)

/* RX1_ADC_I_DIG_LDO_CTRL */

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MASK   (0x00010000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_SHIFT  (0x00000010U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MASK     (0x000E0000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_SHIFT    (0x00000011U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_RESERVED1_MASK                  (0xFFF00000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_RESERVED1_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_RESERVED1_MAX                   (0x00000FFFU)

#define CSL_AR_RFANACIO_RX1_ADC_I_DIG_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX1_CTRL */

#define CSL_AR_RFANACIO_RX1_CTRL_EN_BIAS_GEN_I_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX1_CTRL_EN_BIAS_GEN_I_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_EN_BIAS_GEN_I_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_EN_BIAS_GEN_I_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_BIAS_CTRL_I_MASK                              (0x00000006U)
#define CSL_AR_RFANACIO_RX1_CTRL_BIAS_CTRL_I_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_RX1_CTRL_BIAS_CTRL_I_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_BIAS_CTRL_I_MAX                               (0x00000003U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MASK              (0x00000008U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_SHIFT             (0x00000003U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MASK                 (0x00000020U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA2_OPAMP_100UA_SHIFT                (0x00000005U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA2_OPAMP_100UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_VBGBYR_LNA_100UA_MASK                   (0x00000040U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_VBGBYR_LNA_100UA_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_VBGBYR_LNA_100UA_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_VBGBYR_LNA_100UA_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_LDO_10UA_MASK                     (0x00000100U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_LDO_10UA_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_LDO_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_LDO_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED0_MASK                                (0x00000200U)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED0_SHIFT                               (0x00000009U)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED0_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED0_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IB_PKDET_SINK_10UA_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_RX1_CTRL_IB_PKDET_SINK_10UA_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_RX1_CTRL_IB_PKDET_SINK_10UA_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IB_PKDET_SINK_10UA_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MASK              (0x00001000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_SHIFT             (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_REF_10UA_MASK                     (0x00002000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_REF_10UA_SHIFT                    (0x0000000DU)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_REF_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_IBIAS_I_IFA_REF_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED1_MASK                                (0xFFFF8000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED1_SHIFT                               (0x0000000FU)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_CTRL_RESERVED1_MAX                                 (0x0001FFFFU)

#define CSL_AR_RFANACIO_RX1_CTRL_RESETVAL                                      (0x00000000U)

/* RX1_FE_TMUX_SPARE_CTRL */

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA1_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA1_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA1_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA2_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA2_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA2_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA3_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA3_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA3_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_VCM_LNA3_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED0_MASK                  (0x0000FFF8U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED0_SHIFT                 (0x00000003U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED0_MAX                   (0x00001FFFU)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_PD_DAC_MASK                  (0x00010000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_PD_DAC_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_PD_DAC_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_PD_DAC_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_MASK                (0x00FE0000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_SHIFT               (0x00000011U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_MAX                 (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_MASK                (0x01000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_IN_MASK                 (0x02000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_IN_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_IN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_IN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_MASK               (0x04000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_SHIFT              (0x0000001AU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_MASK         (0x08000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_SHIFT        (0x0000001BU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_MASK        (0x10000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED1_MASK                  (0xC0000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED1_SHIFT                 (0x0000001EU)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESERVED1_MAX                   (0x00000003U)

#define CSL_AR_RFANACIO_RX1_FE_TMUX_SPARE_CTRL_RESETVAL                        (0x00000000U)

/* RX1_RXFE_CTRL_2 */

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_MIXER_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_MIXER_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_MIXER_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_MIXER_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MASK                  (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_MIXER_SHIFT                 (0x00000001U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_MIXER_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MAX                   (0x00000007U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_MASK               (0x00000010U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_SHIFT              (0x00000004U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_MASK       (0x00000FE0U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_RESERVED0_MASK                         (0x00FFF000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_RESERVED0_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_RESERVED0_MAX                          (0x00000FFFU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_SEQ_SELECT_MASK                        (0x03000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_SEQ_SELECT_SHIFT                       (0x00000018U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_SEQ_SELECT_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_SEQ_SELECT_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MASK                  (0xFC000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_IBIAS_BIST_PD_SHIFT                 (0x0000001AU)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_IBIAS_BIST_PD_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_2_RESETVAL                               (0x00000000U)

/* RX1_RXFE_CTRL_1 */

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_RESERVED0_MASK                         (0x0000000EU)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_RESERVED0_SHIFT                        (0x00000001U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_RESERVED0_MAX                          (0x00000007U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MASK              (0x000000F0U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA1_MASK                           (0x00000100U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA1_SHIFT                          (0x00000008U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MASK                   (0x0000FE00U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA1_SHIFT                  (0x00000009U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA2_MASK                           (0x00010000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA2_SHIFT                          (0x00000010U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LNA2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MASK                   (0x00FE0000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA2_SHIFT                  (0x00000011U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LODIST_RX12_BUF1_MASK               (0x01000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LODIST_RX12_BUF1_SHIFT              (0x00000018U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LODIST_RX12_BUF1_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_EN_LODIST_RX12_BUF1_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF1_MASK       (0xFE000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF1_SHIFT      (0x00000019U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF1_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF1_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX1_RXFE_CTRL_1_RESETVAL                               (0x00000000U)

/* LODIST_CTRL3 */

#define CSL_AR_RFANACIO_LODIST_CTRL3_RX12_PWR_SWITCH_EN_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX12_PWR_SWITCH_EN_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX12_PWR_SWITCH_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX12_PWR_SWITCH_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_RX34_PWR_SWITCH_EN_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX34_PWR_SWITCH_EN_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX34_PWR_SWITCH_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RX34_PWR_SWITCH_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_PWR_SWITCH_EN_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_PWR_SWITCH_EN_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_PWR_SWITCH_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_PWR_SWITCH_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_PWR_SWITCH_EN_MASK                   (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_PWR_SWITCH_EN_SHIFT                  (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_PWR_SWITCH_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_PWR_SWITCH_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_SEQ_EN_BYPASS_MASK                   (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_SEQ_EN_BYPASS_SHIFT                  (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_SEQ_EN_BYPASS_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX12_SEQ_EN_BYPASS_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_SEQ_EN_BYPASS_MASK                   (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_SEQ_EN_BYPASS_SHIFT                  (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_SEQ_EN_BYPASS_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_TX34_SEQ_EN_BYPASS_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL3_RESERVED0_MASK                            (0xFFFFFFC0U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RESERVED0_SHIFT                           (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL3_RESERVED0_MAX                             (0x03FFFFFFU)

#define CSL_AR_RFANACIO_LODIST_CTRL3_RESETVAL                                  (0x00000000U)

/* LODIST_ATEST_CTRL3 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_0_MASK           (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_0_SHIFT          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_0_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_1_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_1_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_2_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_2_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_3_MASK           (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_3_SHIFT          (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_3_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_3_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_4_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_4_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_4_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_4_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_5_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_5_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_5_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_5_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_6_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_6_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_6_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_6_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_7_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_7_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_7_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_7_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_8_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_8_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_8_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_8_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_9_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_9_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_9_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_9_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_10_MASK          (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_10_SHIFT         (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_10_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_10_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_11_MASK          (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_11_SHIFT         (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_11_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_11_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_12_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_12_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_12_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_12_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_13_MASK          (0x00002000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_13_SHIFT         (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_13_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_13_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_14_MASK          (0x00004000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_14_SHIFT         (0x0000000EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_14_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_14_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_15_MASK          (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_15_SHIFT         (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_15_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_15_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_16_MASK          (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_16_SHIFT         (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_16_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_16_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_17_MASK          (0x00020000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_17_SHIFT         (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_17_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_17_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_18_MASK          (0x00040000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_18_SHIFT         (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_18_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_18_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_19_MASK          (0x00080000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_19_SHIFT         (0x00000013U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_19_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_19_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_20_MASK          (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_20_SHIFT         (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_20_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_20_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_21_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_21_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_21_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_21_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED0_MASK                      (0x00C00000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED0_SHIFT                     (0x00000016U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED0_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_24_MASK          (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_24_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_24_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_24_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED1_MASK                      (0x3E000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED1_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESERVED1_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_30_MASK          (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_30_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_30_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_30_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_31_MASK          (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_31_SHIFT         (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_31_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RX12_CLUSTER_ATEST_31_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL3_RESETVAL                            (0x00000000U)

/* RX2_RXFE_CTRL_1 */

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_RESERVED0_MASK                         (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_RESERVED0_SHIFT                        (0x00000001U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_RESERVED0_MAX                          (0x00000007U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MASK              (0x000000F0U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA1_MASK                           (0x00000100U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA1_SHIFT                          (0x00000008U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MASK                   (0x0000FE00U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA1_SHIFT                  (0x00000009U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA2_MASK                           (0x00010000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA2_SHIFT                          (0x00000010U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LNA2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MASK                   (0x00FE0000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA2_SHIFT                  (0x00000011U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LODIST_RX12_BUF2_MASK               (0x01000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LODIST_RX12_BUF2_SHIFT              (0x00000018U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LODIST_RX12_BUF2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_EN_LODIST_RX12_BUF2_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF2_MASK       (0xFE000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF2_SHIFT      (0x00000019U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF2_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX12_BUF2_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_1_RESETVAL                               (0x00000000U)

/* RX2_RXFE_CTRL_2 */

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_MIXER_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_MIXER_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_MIXER_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_MIXER_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MASK                  (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_MIXER_SHIFT                 (0x00000001U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_MIXER_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MAX                   (0x00000007U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_SPARE_MASK         (0x00000010U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_SPARE_SHIFT        (0x00000004U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_SPARE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_LODIST_RX12_BUF2_SPARE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_SPARE_MASK (0x00000FE0U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_SPARE_SHIFT (0x00000005U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_SPARE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX12_BUF2_SPARE_MAX  (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED0_MASK                         (0x00FFF000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED0_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED0_MAX                          (0x00000FFFU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_SEQ_SELECT_MASK                        (0x03000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_SEQ_SELECT_SHIFT                       (0x00000018U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_SEQ_SELECT_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_SEQ_SELECT_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED1_MASK                         (0x0C000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED1_SHIFT                        (0x0000001AU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED1_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESERVED1_MAX                          (0x00000003U)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MASK                  (0xF0000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_IBIAS_BIST_PD_SHIFT                 (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_IBIAS_BIST_PD_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_RXFE_CTRL_2_RESETVAL                               (0x00000000U)

/* RX2_FE_TMUX_SPARE_CTRL */

#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA1_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA1_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA1_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA2_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA2_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA2_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA3_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA3_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA3_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_VCM_LNA3_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_RESERVED0_MASK                  (0xFFFFFFF8U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_RESERVED0_SHIFT                 (0x00000003U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_RESERVED0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_RESERVED0_MAX                   (0x1FFFFFFFU)

#define CSL_AR_RFANACIO_RX2_FE_TMUX_SPARE_CTRL_RESETVAL                        (0x00000000U)

/* RX2_CTRL */

#define CSL_AR_RFANACIO_RX2_CTRL_EN_BIAS_GEN_I_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX2_CTRL_EN_BIAS_GEN_I_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_EN_BIAS_GEN_I_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_EN_BIAS_GEN_I_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_BIAS_CTRL_I_MASK                              (0x00000006U)
#define CSL_AR_RFANACIO_RX2_CTRL_BIAS_CTRL_I_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_RX2_CTRL_BIAS_CTRL_I_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_BIAS_CTRL_I_MAX                               (0x00000003U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MASK              (0x00000008U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_SHIFT             (0x00000003U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MASK                 (0x00000020U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA2_OPAMP_100UA_SHIFT                (0x00000005U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA2_OPAMP_100UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_VBGBYR_LNA_100UA_MASK                   (0x00000040U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_VBGBYR_LNA_100UA_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_VBGBYR_LNA_100UA_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_VBGBYR_LNA_100UA_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_LDO_10UA_MASK                     (0x00000100U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_LDO_10UA_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_LDO_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_LDO_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED0_MASK                                (0x00000200U)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED0_SHIFT                               (0x00000009U)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED0_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED0_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IB_PKDET_SINK_10UA_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_RX2_CTRL_IB_PKDET_SINK_10UA_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_RX2_CTRL_IB_PKDET_SINK_10UA_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IB_PKDET_SINK_10UA_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MASK              (0x00001000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_SHIFT             (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_REF_10UA_MASK                     (0x00002000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_REF_10UA_SHIFT                    (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_REF_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_IBIAS_I_IFA_REF_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED1_MASK                                (0xFFFF8000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED1_SHIFT                               (0x0000000FU)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_CTRL_RESERVED1_MAX                                 (0x0001FFFFU)

#define CSL_AR_RFANACIO_RX2_CTRL_RESETVAL                                      (0x00000000U)

/* RX2_ADC_I_DIG_LDO_CTRL */

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MASK   (0x00010000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_SHIFT  (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MASK     (0x000E0000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_SHIFT    (0x00000011U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_RESERVED1_MASK                  (0xFFF00000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_RESERVED1_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_RESERVED1_MAX                   (0x00000FFFU)

#define CSL_AR_RFANACIO_RX2_ADC_I_DIG_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX2_ADC_I_TMUX_CTRL */

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MASK              (0x00000004U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_SHIFT             (0x00000002U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MASK             (0x00000008U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_SHIFT            (0x00000003U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MASK              (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MASK               (0x00000020U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_SHIFT              (0x00000005U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MASK                 (0x00000200U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_SHIFT                (0x00000009U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MASK             (0x00000400U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_SHIFT            (0x0000000AU)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_VSSA_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_VSSA_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_VSSA_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_VSSA_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MASK                  (0x00002000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_SHIFT                 (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_RESERVED0_MASK                     (0xFFFFC000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_RESERVED0_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_RESERVED0_MAX                      (0x0003FFFFU)

#define CSL_AR_RFANACIO_RX2_ADC_I_TMUX_CTRL_RESETVAL                           (0x00000000U)

/* RX2_ADC_I_CTRL_3 */

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MASK        (0x0000000FU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_SHIFT       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MASK                 (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_SHIFT                (0x00000004U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MASK              (0x00000020U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_SHIFT             (0x00000005U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MASK              (0x000003C0U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MASK        (0x00000C00U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_SHIFT       (0x0000000AU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MASK           (0x00003000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MASK           (0x0000C000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MASK              (0x00080000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_SHIFT             (0x00000013U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MASK              (0x00100000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_SHIFT             (0x00000014U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MASK                 (0x00200000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_SHIFT                (0x00000015U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MASK             (0x00400000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_SHIFT            (0x00000016U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MASK           (0x00800000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_SHIFT          (0x00000017U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MASK                   (0x01000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_IFA_CAL_CLK_EN_SHIFT                  (0x00000018U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_IFA_CAL_CLK_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_EN_MASK                     (0x02000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_EN_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_IBIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MASK            (0x04000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_SHIFT           (0x0000001AU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MASK            (0x08000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_SHIFT           (0x0000001BU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MASK              (0x10000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_SHIFT             (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EN_MASK                    (0x20000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EN_SHIFT                   (0x0000001DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_TX_DAC_CLK_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MASK             (0xC0000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_SHIFT            (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MAX              (0x00000003U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_3_RESETVAL                              (0x00000000U)

/* RX2_ADC_I_CTRL_2 */

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MASK             (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MAX              (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MASK       (0x000000F0U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MASK       (0x00000F00U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_SHIFT      (0x00000008U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MASK       (0x0000F000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MASK       (0x000F0000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_SHIFT      (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MASK       (0x00F00000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MASK       (0x0F000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_SHIFT      (0x00000018U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MASK        (0xF0000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_2_RESETVAL                              (0x00000000U)

/* RX2_ADC_I_CTRL_1 */

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MASK               (0x00000007U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MASK               (0x00000038U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MASK               (0x000001C0U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_SHIFT              (0x00000006U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MASK                (0x00000E00U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_SHIFT               (0x00000009U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MASK             (0x00002000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MASK             (0x00008000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_SHIFT            (0x0000000FU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MASK           (0x00380000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_SHIFT          (0x00000013U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MASK              (0x00400000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_SHIFT             (0x00000016U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MASK          (0x03800000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MASK           (0x1C000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_SHIFT          (0x0000001AU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MASK              (0x20000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_SHIFT             (0x0000001DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MASK          (0xC0000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MAX           (0x00000003U)

#define CSL_AR_RFANACIO_RX2_ADC_I_CTRL_1_RESETVAL                              (0x00000000U)

/* RX2_ADC_I_ANA_LDO_CTRL */

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_RESERVED1_MASK                  (0xFFFF0000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_RESERVED1_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_RESERVED1_MAX                   (0x0000FFFFU)

#define CSL_AR_RFANACIO_RX2_ADC_I_ANA_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX2_I_IFA_TEST_CTRL */

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MASK        (0x00000002U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_SHIFT       (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MASK             (0x00000004U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_SHIFT            (0x00000002U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MASK          (0x00000008U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_SHIFT         (0x00000003U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_INPUT_TEST_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_INPUT_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED0_MASK                     (0x00000020U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED0_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED0_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MASK         (0x00000040U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_SHIFT        (0x00000006U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MASK         (0x00000080U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_SHIFT        (0x00000007U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED1_MASK                     (0x00000F00U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED1_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED1_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MASK    (0x00002000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_SHIFT   (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MASK      (0x00004000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_SHIFT     (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MASK    (0x00010000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MASK       (0x00040000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_SHIFT      (0x00000012U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MASK         (0x00100000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_SHIFT        (0x00000014U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MASK          (0x00800000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MASK (0x01000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED2_MASK                     (0xFE000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED2_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESERVED2_MAX                      (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_TEST_CTRL_RESETVAL                           (0x00000000U)

/* RX2_I_IFA_CTRL_5 */

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MASK           (0xF0000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_SHIFT          (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_5_RESETVAL                              (0x00000000U)

/* RX2_I_IFA_CTRL_4 */

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MASK                   (0x0000003FU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_CTRIM_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MASK                   (0x00000FC0U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_CTRIM_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_SEL_MASK                     (0x00007000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_SEL_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFA_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_SEL_MASK                     (0x00038000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_SEL_SHIFT                    (0x0000000FU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPFB_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_R1B_RTRIM_MASK                    (0x00FC0000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_R1B_RTRIM_SHIFT                   (0x00000012U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_R1B_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_R1B_RTRIM_MAX                     (0x0000003FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MASK          (0x07000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MASK           (0x18000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_SHIFT          (0x0000001BU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MASK       (0x20000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_SHIFT      (0x0000001DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_4_RESETVAL                              (0x00000000U)

/* RX2_I_IFA_CTRL_3 */

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGA_RTRIM_MASK                    (0x0000007FU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGA_RTRIM_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGA_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGA_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGB_RTRIM_MASK                    (0x00003F80U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGB_RTRIM_SHIFT                   (0x00000007U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGB_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_RGB_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF_CTRL_MASK                     (0x003FC000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF_CTRL_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF_CTRL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF_CTRL_MAX                      (0x000000FFU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MASK               (0x00C00000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_SHIFT              (0x00000016U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MASK                 (0x06000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MAX                  (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_REDERVED0_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_REDERVED0_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_REDERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_REDERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MASK                 (0x10000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_SHIFT                (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_RESERVED0_MASK                        (0xC0000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_RESERVED0_SHIFT                       (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_RESERVED0_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_3_RESETVAL                              (0x00000000U)

/* RX2_I_IFA_CTRL_2 */

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_EN_MASK                    (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_EN_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MASK              (0x0000007EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_SHIFT             (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_EN_MASK                    (0x00000100U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_EN_SHIFT                   (0x00000008U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MASK                  (0x0000FC00U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_CTRL_SHIFT                 (0x0000000AU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_CTRL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_EN_MASK                   (0x00010000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_EN_SHIFT                  (0x00000010U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MASK           (0x00020000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_SHIFT          (0x00000011U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MASK                 (0x00040000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_SHIFT                (0x00000012U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_EN_MUX_SEL_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_EN_MUX_SEL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MASK       (0xFF800000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_SHIFT      (0x00000017U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MAX        (0x000001FFU)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_2_RESETVAL                              (0x00000000U)

/* RX2_I_IFA_REFGEN_LDO_CTRL */

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MASK       (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_SHIFT      (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MASK      (0x00000020U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_SHIFT     (0x00000005U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MASK     (0x00000040U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_SHIFT    (0x00000006U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MASK       (0x00000080U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_SHIFT      (0x00000007U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MASK         (0x00000700U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MAX          (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MASK      (0x0000E000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_SHIFT     (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MAX       (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MASK           (0x00010000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_SHIFT          (0x00000010U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MASK (0x000E0000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MASK (0x00700000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_SHIFT (0x00000014U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MAX  (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MASK (0x03800000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_SHIFT (0x00000017U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MASK (0x1C000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_SHIFT (0x0000001AU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MASK (0xE0000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_SHIFT (0x0000001DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_REFGEN_LDO_CTRL_RESETVAL                     (0x00000000U)

/* RX2_I_IFA_CTRL_1 */

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MASK                  (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MASK          (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED0_MASK                        (0x00000020U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED0_SHIFT                       (0x00000005U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MASK        (0x000000C0U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_SHIFT       (0x00000006U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MASK                   (0x00001000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_EN_SHIFT                  (0x0000000CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MASK              (0x00002000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_SHIFT             (0x0000000DU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MASK           (0x0001C000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED1_MASK                        (0x00020000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED1_SHIFT                       (0x00000011U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED1_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED1_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MASK         (0x000C0000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_SHIFT        (0x00000012U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MASK       (0x00300000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MASK                   (0x00400000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_EN_SHIFT                  (0x00000016U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MASK              (0x00800000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_SHIFT             (0x00000017U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MASK           (0x07000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_SHIFT          (0x00000018U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED2_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED2_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED2_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESERVED2_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MASK         (0x30000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_SHIFT        (0x0000001CU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX2_I_IFA_CTRL_1_RESETVAL                              (0x00000000U)

/* RX_TOPMUX_BUF_CTRL */

#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_IFA_CALIB_CLK_EN_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_IFA_CALIB_CLK_EN_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_IFA_CALIB_CLK_EN_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_IFA_CALIB_CLK_EN_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_TOPMUX_BUF_CTRL_MASK             (0xFFFFFFFEU)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_TOPMUX_BUF_CTRL_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_TOPMUX_BUF_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RX_TOPMUX_BUF_CTRL_MAX              (0x7FFFFFFFU)

#define CSL_AR_RFANACIO_RX_TOPMUX_BUF_CTRL_RESETVAL                            (0x00000000U)

/* RX_TOP_ADC_PWRUP_BYPASS_CTRL */

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX1_ADC_I_PWRUP_BYPASS_CTRL_MASK (0x00000007U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX1_ADC_I_PWRUP_BYPASS_CTRL_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX1_ADC_I_PWRUP_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX1_ADC_I_PWRUP_BYPASS_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED0_MASK            (0x000000F8U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED0_SHIFT           (0x00000003U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED0_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED0_MAX             (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX2_ADC_I_PWRUP_BYPASS_CTRL_MASK (0x00000700U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX2_ADC_I_PWRUP_BYPASS_CTRL_SHIFT (0x00000008U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX2_ADC_I_PWRUP_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX2_ADC_I_PWRUP_BYPASS_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED1_MASK            (0x0000F800U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED1_SHIFT           (0x0000000BU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED1_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED1_MAX             (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX3_ADC_I_PWRUP_BYPASS_CTRL_MASK (0x00070000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX3_ADC_I_PWRUP_BYPASS_CTRL_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX3_ADC_I_PWRUP_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX3_ADC_I_PWRUP_BYPASS_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED2_MASK            (0x00F80000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED2_SHIFT           (0x00000013U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED2_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED2_MAX             (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX4_ADC_I_PWRUP_BYPASS_CTRL_MASK (0x07000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX4_ADC_I_PWRUP_BYPASS_CTRL_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX4_ADC_I_PWRUP_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX4_ADC_I_PWRUP_BYPASS_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED3_MASK            (0x38000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED3_SHIFT           (0x0000001BU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED3_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESERVED3_MAX             (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX_ADC_CLK_BYPASS_CTRL_MASK (0xC0000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX_ADC_CLK_BYPASS_CTRL_SHIFT (0x0000001EU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX_ADC_CLK_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RX_ADC_CLK_BYPASS_CTRL_MAX (0x00000003U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_CTRL_RESETVAL                  (0x00000000U)

/* RX_TOP_ADC_PWRUP_BYPASS_EN */

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX1_ADC_I_PWRUP_BYPASS_EN_MASK (0x00000007U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX1_ADC_I_PWRUP_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX1_ADC_I_PWRUP_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX1_ADC_I_PWRUP_BYPASS_EN_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED0_MASK              (0x000000F8U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED0_SHIFT             (0x00000003U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED0_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED0_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX2_ADC_I_PWRUP_BYPASS_EN_MASK (0x00000700U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX2_ADC_I_PWRUP_BYPASS_EN_SHIFT (0x00000008U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX2_ADC_I_PWRUP_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX2_ADC_I_PWRUP_BYPASS_EN_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED1_MASK              (0x0000F800U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED1_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED1_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX3_ADC_I_PWRUP_BYPASS_EN_MASK (0x00070000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX3_ADC_I_PWRUP_BYPASS_EN_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX3_ADC_I_PWRUP_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX3_ADC_I_PWRUP_BYPASS_EN_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED2_MASK              (0x00F80000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED2_SHIFT             (0x00000013U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED2_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED2_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX4_ADC_I_PWRUP_BYPASS_EN_MASK (0x07000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX4_ADC_I_PWRUP_BYPASS_EN_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX4_ADC_I_PWRUP_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX4_ADC_I_PWRUP_BYPASS_EN_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED3_MASK              (0x38000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED3_SHIFT             (0x0000001BU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED3_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESERVED3_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX_ADC_CLK_BYPASS_EN_MASK   (0xC0000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX_ADC_CLK_BYPASS_EN_SHIFT  (0x0000001EU)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX_ADC_CLK_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RX_ADC_CLK_BYPASS_EN_MAX    (0x00000003U)

#define CSL_AR_RFANACIO_RX_TOP_ADC_PWRUP_BYPASS_EN_RESETVAL                    (0x00000000U)

/* RX_TOP_SPARE_REG1 */

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1_RX_TOP_SPARE_REG1_MASK               (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1_RX_TOP_SPARE_REG1_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1_RX_TOP_SPARE_REG1_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1_RX_TOP_SPARE_REG1_MAX                (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG1_RESETVAL                             (0x00000000U)

/* RX_TOP_SPARE_REG2 */

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2_RX_TOP_SPARE_REG2_MASK               (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2_RX_TOP_SPARE_REG2_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2_RX_TOP_SPARE_REG2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2_RX_TOP_SPARE_REG2_MAX                (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG2_RESETVAL                             (0x00000000U)

/* RX_REFSYS_TMUX_SPARE_CTRL */

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P45V_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P45V_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P45V_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P45V_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_TOP_0P9V_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_TOP_0P9V_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_TOP_0P9V_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_TOP_0P9V_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P9V_FILT_MASK          (0x00000004U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P9V_FILT_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P9V_FILT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P9V_FILT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBE_WEAK_MASK                (0x00000008U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBE_WEAK_SHIFT               (0x00000003U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBE_WEAK_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBE_WEAK_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P22V_MASK               (0x00000010U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P22V_SHIFT              (0x00000004U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P22V_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P22V_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_PRETRIM_0P9V_MASK        (0x00000020U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_PRETRIM_0P9V_SHIFT       (0x00000005U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_PRETRIM_0P9V_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_PRETRIM_0P9V_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_INT_0P9V_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_INT_0P9V_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_INT_0P9V_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_INT_0P9V_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_10U_MASK         (0x00000080U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_10U_SHIFT        (0x00000007U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_10U_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_10U_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P1V_MASK                (0x00000100U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P1V_SHIFT               (0x00000008U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P1V_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VBG_1P1V_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_100U_MASK        (0x00000200U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_100U_SHIFT       (0x00000009U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_100U_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_IBIASP_TRIM_100U_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VSSA_REF_MASK                (0x00000400U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VSSA_REF_SHIFT               (0x0000000AU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VSSA_REF_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VSSA_REF_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P6V_MASK               (0x00000800U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P6V_SHIFT              (0x0000000BU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P6V_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VREF_0P6V_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_I2V_SENSE_MASK               (0x00001000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_I2V_SENSE_SHIFT              (0x0000000CU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_I2V_SENSE_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_I2V_SENSE_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VPBIAS_CURR_MIRR_MASK        (0x00002000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VPBIAS_CURR_MIRR_SHIFT       (0x0000000DU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VPBIAS_CURR_MIRR_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_VPBIAS_CURR_MIRR_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_RESERVED0_MASK               (0x7FFFC000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_RESERVED0_SHIFT              (0x0000000EU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_RESERVED0_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_RESERVED0_MAX                (0x0001FFFFU)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_ANA_TEST_ESD_MUX_EN_MASK     (0x80000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_ANA_TEST_ESD_MUX_EN_SHIFT    (0x0000001FU)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_ANA_TEST_ESD_MUX_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_ANA_TEST_ESD_MUX_EN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX_REFSYS_TMUX_SPARE_CTRL_RESETVAL                     (0x00000000U)

/* RX3_I_IFA_CTRL_1 */

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MASK                  (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MASK          (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED0_MASK                        (0x00000020U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED0_SHIFT                       (0x00000005U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MASK        (0x000000C0U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_SHIFT       (0x00000006U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MASK                   (0x00001000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_EN_SHIFT                  (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MASK              (0x00002000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_SHIFT             (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MASK           (0x0001C000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED1_MASK                        (0x00020000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED1_SHIFT                       (0x00000011U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED1_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED1_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MASK         (0x000C0000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_SHIFT        (0x00000012U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MASK       (0x00300000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MASK                   (0x00400000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_EN_SHIFT                  (0x00000016U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MASK              (0x00800000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_SHIFT             (0x00000017U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MASK           (0x07000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_SHIFT          (0x00000018U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED2_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED2_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED2_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESERVED2_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MASK         (0x30000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_SHIFT        (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_1_RESETVAL                              (0x00000000U)

/* RX3_I_IFA_REFGEN_LDO_CTRL */

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MASK       (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_SHIFT      (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MASK      (0x00000020U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_SHIFT     (0x00000005U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MASK     (0x00000040U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_SHIFT    (0x00000006U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MASK       (0x00000080U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_SHIFT      (0x00000007U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MASK         (0x00000700U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MAX          (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MASK      (0x0000E000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_SHIFT     (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MAX       (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MASK           (0x00010000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_SHIFT          (0x00000010U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MASK (0x000E0000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MASK (0x00700000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_SHIFT (0x00000014U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MAX  (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MASK (0x03800000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_SHIFT (0x00000017U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MASK (0x1C000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_SHIFT (0x0000001AU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MASK (0xE0000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_SHIFT (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_REFGEN_LDO_CTRL_RESETVAL                     (0x00000000U)

/* RX3_I_IFA_CTRL_2 */

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_EN_MASK                    (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_EN_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MASK              (0x0000007EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_SHIFT             (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_EN_MASK                    (0x00000100U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_EN_SHIFT                   (0x00000008U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MASK                  (0x0000FC00U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_CTRL_SHIFT                 (0x0000000AU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_CTRL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_EN_MASK                   (0x00010000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_EN_SHIFT                  (0x00000010U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MASK           (0x00020000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_SHIFT          (0x00000011U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MASK                 (0x00040000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_SHIFT                (0x00000012U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_EN_MUX_SEL_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_EN_MUX_SEL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MASK       (0xFF800000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_SHIFT      (0x00000017U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MAX        (0x000001FFU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_2_RESETVAL                              (0x00000000U)

/* RX3_I_IFA_CTRL_3 */

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGA_RTRIM_MASK                    (0x0000007FU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGA_RTRIM_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGA_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGA_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGB_RTRIM_MASK                    (0x00003F80U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGB_RTRIM_SHIFT                   (0x00000007U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGB_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_RGB_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF_CTRL_MASK                     (0x003FC000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF_CTRL_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF_CTRL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF_CTRL_MAX                      (0x000000FFU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MASK               (0x00C00000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_SHIFT              (0x00000016U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MASK                 (0x06000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MAX                  (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_REDERVED0_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_REDERVED0_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_REDERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_REDERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MASK                 (0x10000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_SHIFT                (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_RESERVED0_MASK                        (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_RESERVED0_SHIFT                       (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_RESERVED0_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_3_RESETVAL                              (0x00000000U)

/* RX3_I_IFA_CTRL_4 */

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MASK                   (0x0000003FU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_CTRIM_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MASK                   (0x00000FC0U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_CTRIM_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_SEL_MASK                     (0x00007000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_SEL_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFA_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_SEL_MASK                     (0x00038000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_SEL_SHIFT                    (0x0000000FU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPFB_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_R1B_RTRIM_MASK                    (0x00FC0000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_R1B_RTRIM_SHIFT                   (0x00000012U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_R1B_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_R1B_RTRIM_MAX                     (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MASK          (0x07000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MASK           (0x18000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_SHIFT          (0x0000001BU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MASK       (0x20000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_SHIFT      (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_4_RESETVAL                              (0x00000000U)

/* RX3_I_IFA_CTRL_5 */

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MASK           (0xF0000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_SHIFT          (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_CTRL_5_RESETVAL                              (0x00000000U)

/* RX3_I_IFA_TEST_CTRL */

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MASK        (0x00000002U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_SHIFT       (0x00000001U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MASK             (0x00000004U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_SHIFT            (0x00000002U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MASK          (0x00000008U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_SHIFT         (0x00000003U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_INPUT_TEST_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_INPUT_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED0_MASK                     (0x00000020U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED0_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED0_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MASK         (0x00000040U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_SHIFT        (0x00000006U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MASK         (0x00000080U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_SHIFT        (0x00000007U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED1_MASK                     (0x00000F00U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED1_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED1_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MASK    (0x00002000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_SHIFT   (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MASK      (0x00004000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_SHIFT     (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MASK    (0x00010000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MASK       (0x00040000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_SHIFT      (0x00000012U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MASK         (0x00100000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_SHIFT        (0x00000014U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MASK          (0x00800000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MASK (0x01000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED2_MASK                     (0xFE000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED2_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESERVED2_MAX                      (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_I_IFA_TEST_CTRL_RESETVAL                           (0x00000000U)

/* RX3_ADC_I_ANA_LDO_CTRL */

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_RESERVED1_MASK                  (0xFFFF0000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_RESERVED1_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_RESERVED1_MAX                   (0x0000FFFFU)

#define CSL_AR_RFANACIO_RX3_ADC_I_ANA_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX3_ADC_I_CTRL_1 */

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MASK               (0x00000007U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MASK               (0x00000038U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MASK               (0x000001C0U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_SHIFT              (0x00000006U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MASK                (0x00000E00U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_SHIFT               (0x00000009U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MASK             (0x00002000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MASK             (0x00008000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_SHIFT            (0x0000000FU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MASK           (0x00380000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_SHIFT          (0x00000013U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MASK              (0x00400000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_SHIFT             (0x00000016U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MASK          (0x03800000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MASK           (0x1C000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_SHIFT          (0x0000001AU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MASK              (0x20000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_SHIFT             (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MASK          (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MAX           (0x00000003U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_1_RESETVAL                              (0x00000000U)

/* RX3_ADC_I_CTRL_2 */

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MASK             (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MAX              (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MASK       (0x000000F0U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MASK       (0x00000F00U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_SHIFT      (0x00000008U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MASK       (0x0000F000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MASK       (0x000F0000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_SHIFT      (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MASK       (0x00F00000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MASK       (0x0F000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_SHIFT      (0x00000018U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MASK        (0xF0000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_2_RESETVAL                              (0x00000000U)

/* RX3_ADC_I_CTRL_3 */

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MASK        (0x0000000FU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_SHIFT       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MASK                 (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_SHIFT                (0x00000004U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MASK              (0x00000020U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_SHIFT             (0x00000005U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MASK              (0x000003C0U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MASK        (0x00000C00U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_SHIFT       (0x0000000AU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MASK           (0x00003000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MASK           (0x0000C000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MASK              (0x00080000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_SHIFT             (0x00000013U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MASK              (0x00100000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_SHIFT             (0x00000014U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MASK                 (0x00200000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_SHIFT                (0x00000015U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MASK             (0x00400000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_SHIFT            (0x00000016U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MASK           (0x00800000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_SHIFT          (0x00000017U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MASK                   (0x01000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_IFA_CAL_CLK_EN_SHIFT                  (0x00000018U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_IFA_CAL_CLK_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_EN_MASK                     (0x02000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_EN_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_IBIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MASK            (0x04000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_SHIFT           (0x0000001AU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MASK            (0x08000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_SHIFT           (0x0000001BU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MASK              (0x10000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_SHIFT             (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EN_MASK                    (0x20000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EN_SHIFT                   (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_TX_DAC_CLK_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MASK             (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_SHIFT            (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MAX              (0x00000003U)

#define CSL_AR_RFANACIO_RX3_ADC_I_CTRL_3_RESETVAL                              (0x00000000U)

/* RX3_ADC_I_TMUX_CTRL */

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MASK              (0x00000004U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_SHIFT             (0x00000002U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MASK             (0x00000008U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_SHIFT            (0x00000003U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MASK              (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MASK               (0x00000020U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_SHIFT              (0x00000005U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MASK                 (0x00000200U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_SHIFT                (0x00000009U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MASK             (0x00000400U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_SHIFT            (0x0000000AU)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_VSSA_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_VSSA_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_VSSA_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_VSSA_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MASK                  (0x00002000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_SHIFT                 (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_RESERVED0_MASK                     (0xFFFFC000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_RESERVED0_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_RESERVED0_MAX                      (0x0003FFFFU)

#define CSL_AR_RFANACIO_RX3_ADC_I_TMUX_CTRL_RESETVAL                           (0x00000000U)

/* RX3_ADC_I_DIG_LDO_CTRL */

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MASK   (0x00010000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_SHIFT  (0x00000010U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MASK     (0x000E0000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_SHIFT    (0x00000011U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_RESERVED1_MASK                  (0xFFF00000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_RESERVED1_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_RESERVED1_MAX                   (0x00000FFFU)

#define CSL_AR_RFANACIO_RX3_ADC_I_DIG_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX3_CTRL */

#define CSL_AR_RFANACIO_RX3_CTRL_EN_BIAS_GEN_I_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX3_CTRL_EN_BIAS_GEN_I_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_EN_BIAS_GEN_I_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_EN_BIAS_GEN_I_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_BIAS_CTRL_I_MASK                              (0x00000006U)
#define CSL_AR_RFANACIO_RX3_CTRL_BIAS_CTRL_I_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_RX3_CTRL_BIAS_CTRL_I_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_BIAS_CTRL_I_MAX                               (0x00000003U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MASK              (0x00000008U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_SHIFT             (0x00000003U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MASK                 (0x00000020U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA2_OPAMP_100UA_SHIFT                (0x00000005U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA2_OPAMP_100UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_VBGBYR_LNA_100UA_MASK                   (0x00000040U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_VBGBYR_LNA_100UA_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_VBGBYR_LNA_100UA_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_VBGBYR_LNA_100UA_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_LDO_10UA_MASK                     (0x00000100U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_LDO_10UA_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_LDO_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_LDO_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED0_MASK                                (0x00000200U)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED0_SHIFT                               (0x00000009U)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED0_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED0_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IB_PKDET_SINK_10UA_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_RX3_CTRL_IB_PKDET_SINK_10UA_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_RX3_CTRL_IB_PKDET_SINK_10UA_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IB_PKDET_SINK_10UA_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MASK              (0x00001000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_SHIFT             (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_REF_10UA_MASK                     (0x00002000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_REF_10UA_SHIFT                    (0x0000000DU)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_REF_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_IBIAS_I_IFA_REF_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED1_MASK                                (0xFFFF8000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED1_SHIFT                               (0x0000000FU)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_CTRL_RESERVED1_MAX                                 (0x0001FFFFU)

#define CSL_AR_RFANACIO_RX3_CTRL_RESETVAL                                      (0x00000000U)

/* RX3_FE_TMUX_SPARE_CTRL */

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA1_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA1_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA1_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA2_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA2_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA2_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA3_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA3_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA3_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_VCM_LNA3_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED0_MASK                  (0x0000FFF8U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED0_SHIFT                 (0x00000003U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED0_MAX                   (0x00001FFFU)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_PD_DAC_MASK                  (0x00010000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_PD_DAC_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_PD_DAC_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_PD_DAC_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_MASK                (0x00FE0000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_SHIFT               (0x00000011U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PD_DAC_CTRL_MAX                 (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_MASK                (0x01000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_OUT_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_IN_MASK                 (0x02000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_IN_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_IN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_IN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_MASK               (0x04000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_SHIFT              (0x0000001AU)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_STG1_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_MASK         (0x08000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_SHIFT        (0x0000001BU)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_VREF_0P45V_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_MASK        (0x10000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_EN_TEST_ITEST_100UA_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_PDLNA_IBIAS_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED1_MASK                  (0xC0000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED1_SHIFT                 (0x0000001EU)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESERVED1_MAX                   (0x00000003U)

#define CSL_AR_RFANACIO_RX3_FE_TMUX_SPARE_CTRL_RESETVAL                        (0x00000000U)

/* RX3_RXFE_CTRL_2 */

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_MIXER_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_MIXER_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_MIXER_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_MIXER_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MASK                  (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_MIXER_SHIFT                 (0x00000001U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_MIXER_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MAX                   (0x00000007U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_MASK               (0x00000010U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_SHIFT              (0x00000004U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_MASK       (0x00000FE0U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_RESERVED0_MASK                         (0x00FFF000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_RESERVED0_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_RESERVED0_MAX                          (0x00000FFFU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_SEQ_SELECT_MASK                        (0x03000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_SEQ_SELECT_SHIFT                       (0x00000018U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_SEQ_SELECT_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_SEQ_SELECT_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MASK                  (0xFC000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_IBIAS_BIST_PD_SHIFT                 (0x0000001AU)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_IBIAS_BIST_PD_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_2_RESETVAL                               (0x00000000U)

/* RX3_RXFE_CTRL_1 */

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_RESERVED0_MASK                         (0x0000000EU)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_RESERVED0_SHIFT                        (0x00000001U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_RESERVED0_MAX                          (0x00000007U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MASK              (0x000000F0U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA1_MASK                           (0x00000100U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA1_SHIFT                          (0x00000008U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MASK                   (0x0000FE00U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA1_SHIFT                  (0x00000009U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA2_MASK                           (0x00010000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA2_SHIFT                          (0x00000010U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LNA2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MASK                   (0x00FE0000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA2_SHIFT                  (0x00000011U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LODIST_RX34_BUF1_MASK               (0x01000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LODIST_RX34_BUF1_SHIFT              (0x00000018U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LODIST_RX34_BUF1_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_EN_LODIST_RX34_BUF1_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF1_MASK       (0xFE000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF1_SHIFT      (0x00000019U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF1_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF1_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX3_RXFE_CTRL_1_RESETVAL                               (0x00000000U)

/* LODIST_ATEST_CTRL4 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_0_MASK           (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_0_SHIFT          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_0_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_1_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_1_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_2_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_2_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_3_MASK           (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_3_SHIFT          (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_3_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_3_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_4_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_4_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_4_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_4_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_5_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_5_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_5_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_5_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_6_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_6_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_6_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_6_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_7_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_7_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_7_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_7_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_8_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_8_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_8_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_8_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_9_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_9_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_9_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_9_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_10_MASK          (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_10_SHIFT         (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_10_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_10_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_11_MASK          (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_11_SHIFT         (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_11_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_11_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_12_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_12_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_12_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_12_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_13_MASK          (0x00002000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_13_SHIFT         (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_13_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_13_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_14_MASK          (0x00004000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_14_SHIFT         (0x0000000EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_14_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_14_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_15_MASK          (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_15_SHIFT         (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_15_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_15_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_16_MASK          (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_16_SHIFT         (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_16_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_16_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_17_MASK          (0x00020000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_17_SHIFT         (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_17_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_17_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_18_MASK          (0x00040000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_18_SHIFT         (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_18_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_18_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_19_MASK          (0x00080000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_19_SHIFT         (0x00000013U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_19_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_19_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_20_MASK          (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_20_SHIFT         (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_20_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_20_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_21_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_21_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_21_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_21_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED0_MASK                      (0x00C00000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED0_SHIFT                     (0x00000016U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED0_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_24_MASK          (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_24_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_24_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_24_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED1_MASK                      (0x3E000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED1_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESERVED1_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_30_MASK          (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_30_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_30_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_30_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_31_MASK          (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_31_SHIFT         (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_31_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RX34_CLUSTER_ATEST_31_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL4_RESETVAL                            (0x00000000U)

/* LODIST_CTRL4 */

#define CSL_AR_RFANACIO_LODIST_CTRL4_RESERVED0_MASK                            (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_LODIST_CTRL4_RESERVED0_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL4_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL4_RESERVED0_MAX                             (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_LODIST_CTRL4_RESETVAL                                  (0x00000000U)

/* RX4_RXFE_CTRL_1 */

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_RESERVED0_MASK                         (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_RESERVED0_SHIFT                        (0x00000001U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_RESERVED0_MAX                          (0x00000007U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MASK              (0x000000F0U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_HI_LINEARITY_MODE_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA1_MASK                           (0x00000100U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA1_SHIFT                          (0x00000008U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MASK                   (0x0000FE00U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA1_SHIFT                  (0x00000009U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA1_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA2_MASK                           (0x00010000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA2_SHIFT                          (0x00000010U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LNA2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MASK                   (0x00FE0000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA2_SHIFT                  (0x00000011U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LNA2_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LODIST_RX34_BUF2_MASK               (0x01000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LODIST_RX34_BUF2_SHIFT              (0x00000018U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LODIST_RX34_BUF2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_EN_LODIST_RX34_BUF2_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF2_MASK       (0xFE000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF2_SHIFT      (0x00000019U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF2_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_CTRL_IBIAS_LODIST_RX34_BUF2_MAX        (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_1_RESETVAL                               (0x00000000U)

/* RX4_RXFE_CTRL_2 */

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_MIXER_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_MIXER_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_MIXER_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_MIXER_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MASK                  (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_MIXER_SHIFT                 (0x00000001U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_MIXER_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_MIXER_MAX                   (0x00000007U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_SPARE_MASK         (0x00000010U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_SPARE_SHIFT        (0x00000004U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_SPARE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_LODIST_RX34_BUF2_SPARE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_SPARE_MASK (0x00000FE0U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_SPARE_SHIFT (0x00000005U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_SPARE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_CTRL_IBIAS_LODIST_RX34_BUF2_SPARE_MAX  (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED0_MASK                         (0x00FFF000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED0_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED0_MAX                          (0x00000FFFU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_SEQ_SELECT_MASK                        (0x03000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_SEQ_SELECT_SHIFT                       (0x00000018U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_SEQ_SELECT_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_SEQ_SELECT_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED1_MASK                         (0x0C000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED1_SHIFT                        (0x0000001AU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED1_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESERVED1_MAX                          (0x00000003U)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MASK                  (0xF0000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_IBIAS_BIST_PD_SHIFT                 (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_IBIAS_BIST_PD_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_EN_IBIAS_BIST_PD_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_RXFE_CTRL_2_RESETVAL                               (0x00000000U)

/* RX4_FE_TMUX_SPARE_CTRL */

#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA1_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA1_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA1_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA2_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA2_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA2_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA3_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA3_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA3_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_VCM_LNA3_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_RESERVED0_MASK                  (0xFFFFFFF8U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_RESERVED0_SHIFT                 (0x00000003U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_RESERVED0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_RESERVED0_MAX                   (0x1FFFFFFFU)

#define CSL_AR_RFANACIO_RX4_FE_TMUX_SPARE_CTRL_RESETVAL                        (0x00000000U)

/* RX4_CTRL */

#define CSL_AR_RFANACIO_RX4_CTRL_EN_BIAS_GEN_I_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_RX4_CTRL_EN_BIAS_GEN_I_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_EN_BIAS_GEN_I_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_EN_BIAS_GEN_I_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_BIAS_CTRL_I_MASK                              (0x00000006U)
#define CSL_AR_RFANACIO_RX4_CTRL_BIAS_CTRL_I_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_RX4_CTRL_BIAS_CTRL_I_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_BIAS_CTRL_I_MAX                               (0x00000003U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MASK              (0x00000008U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_SHIFT             (0x00000003U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_MAIN_AMP_100UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA1_FB_AMP_100UA_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MASK                 (0x00000020U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA2_OPAMP_100UA_SHIFT                (0x00000005U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA2_OPAMP_100UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA2_OPAMP_100UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_VBGBYR_LNA_100UA_MASK                   (0x00000040U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_VBGBYR_LNA_100UA_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_VBGBYR_LNA_100UA_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_VBGBYR_LNA_100UA_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_MAIN_AMP_CAS_100UA_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_LDO_10UA_MASK                     (0x00000100U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_LDO_10UA_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_LDO_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_LDO_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED0_MASK                                (0x00000200U)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED0_SHIFT                               (0x00000009U)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED0_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED0_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IB_PKDET_SINK_10UA_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_RX4_CTRL_IB_PKDET_SINK_10UA_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_RX4_CTRL_IB_PKDET_SINK_10UA_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IB_PKDET_SINK_10UA_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_ANA_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MASK              (0x00001000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_SHIFT             (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_IBIAS_LDO_DIG_10UA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_REF_10UA_MASK                     (0x00002000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_REF_10UA_SHIFT                    (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_REF_10UA_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_IBIAS_I_IFA_REF_10UA_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RX_ADC_I_TRIM_IBIAS_20UA_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED1_MASK                                (0xFFFF8000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED1_SHIFT                               (0x0000000FU)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_CTRL_RESERVED1_MAX                                 (0x0001FFFFU)

#define CSL_AR_RFANACIO_RX4_CTRL_RESETVAL                                      (0x00000000U)

/* RX_TEST_EN */

#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_LNA_TEST_EN_MASK                        (0x00000001U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_LNA_TEST_EN_SHIFT                       (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_LNA_TEST_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_LNA_TEST_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED0_MASK                              (0x00000002U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED0_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED0_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED0_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_IFA1_I_TEST_EN_MASK                     (0x00000004U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_IFA1_I_TEST_EN_SHIFT                    (0x00000002U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_IFA1_I_TEST_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_IFA1_I_TEST_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED1_MASK                              (0x00000008U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED1_SHIFT                             (0x00000003U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED1_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED1_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_ADC_I_TEST_EN_MASK                      (0x00000010U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_ADC_I_TEST_EN_SHIFT                     (0x00000004U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_ADC_I_TEST_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX1_ADC_I_TEST_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED2_MASK                              (0x000000E0U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED2_SHIFT                             (0x00000005U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED2_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED2_MAX                               (0x00000007U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_LNA_TEST_EN_MASK                        (0x00000100U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_LNA_TEST_EN_SHIFT                       (0x00000008U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_LNA_TEST_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_LNA_TEST_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED3_MASK                              (0x00000200U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED3_SHIFT                             (0x00000009U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED3_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED3_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_IFA1_I_TEST_EN_MASK                     (0x00000400U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_IFA1_I_TEST_EN_SHIFT                    (0x0000000AU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_IFA1_I_TEST_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_IFA1_I_TEST_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED4_MASK                              (0x00000800U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED4_SHIFT                             (0x0000000BU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED4_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED4_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_ADC_I_TEST_EN_MASK                      (0x00001000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_ADC_I_TEST_EN_SHIFT                     (0x0000000CU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_ADC_I_TEST_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX2_ADC_I_TEST_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED5_MASK                              (0x0000E000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED5_SHIFT                             (0x0000000DU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED5_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED5_MAX                               (0x00000007U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_LNA_TEST_EN_MASK                        (0x00010000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_LNA_TEST_EN_SHIFT                       (0x00000010U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_LNA_TEST_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_LNA_TEST_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED6_MASK                              (0x00020000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED6_SHIFT                             (0x00000011U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED6_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED6_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_IFA1_I_TEST_EN_MASK                     (0x00040000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_IFA1_I_TEST_EN_SHIFT                    (0x00000012U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_IFA1_I_TEST_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_IFA1_I_TEST_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED7_MASK                              (0x00080000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED7_SHIFT                             (0x00000013U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED7_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED7_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_ADC_I_TEST_EN_MASK                      (0x00100000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_ADC_I_TEST_EN_SHIFT                     (0x00000014U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_ADC_I_TEST_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX3_ADC_I_TEST_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED8_MASK                              (0x00E00000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED8_SHIFT                             (0x00000015U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED8_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED8_MAX                               (0x00000007U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_LNA_TEST_EN_MASK                        (0x01000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_LNA_TEST_EN_SHIFT                       (0x00000018U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_LNA_TEST_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_LNA_TEST_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED9_MASK                              (0x02000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED9_SHIFT                             (0x00000019U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED9_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED9_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_IFA1_I_TEST_EN_MASK                     (0x04000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_IFA1_I_TEST_EN_SHIFT                    (0x0000001AU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_IFA1_I_TEST_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_IFA1_I_TEST_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED10_MASK                             (0x08000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED10_SHIFT                            (0x0000001BU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED10_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED10_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_ADC_I_TEST_EN_MASK                      (0x10000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_ADC_I_TEST_EN_SHIFT                     (0x0000001CU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_ADC_I_TEST_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RX4_ADC_I_TEST_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED11_MASK                             (0xE0000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED11_SHIFT                            (0x0000001DU)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED11_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_RX_TEST_EN_RESERVED11_MAX                              (0x00000007U)

#define CSL_AR_RFANACIO_RX_TEST_EN_RESETVAL                                    (0x00000000U)

/* RX4_ADC_I_DIG_LDO_CTRL */

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MASK   (0x00010000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_SHIFT  (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_CLK_3P6OR1P8G_CTRL_2_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MASK     (0x000E0000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_SHIFT    (0x00000011U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_ADC_DIG_LDO_ILOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_RESERVED1_MASK                  (0xFFF00000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_RESERVED1_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_RESERVED1_MAX                   (0x00000FFFU)

#define CSL_AR_RFANACIO_RX4_ADC_I_DIG_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX4_ADC_I_TMUX_CTRL */

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MASK              (0x00000004U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_SHIFT             (0x00000002U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MASK             (0x00000008U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_SHIFT            (0x00000003U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_ANA_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MASK              (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VOUT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MASK               (0x00000020U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_SHIFT              (0x00000005U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VIN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_VSSA_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DIG_LDO_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC23_VCM_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MASK                 (0x00000200U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_SHIFT                (0x00000009U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_DAC_ITEST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MASK             (0x00000400U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_SHIFT            (0x0000000AU)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_CNST_GM_ITEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_VSSA_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_VSSA_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_VSSA_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_VSSA_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_SAT_DET_COMP_OUT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MASK                  (0x00002000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_SHIFT                 (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_ADC_INT_RSTZ_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_RESERVED0_MASK                     (0xFFFFC000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_RESERVED0_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_RESERVED0_MAX                      (0x0003FFFFU)

#define CSL_AR_RFANACIO_RX4_ADC_I_TMUX_CTRL_RESETVAL                           (0x00000000U)

/* RX4_ADC_I_CTRL_3 */

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MASK        (0x0000000FU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_SHIFT       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC2_N_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MASK                 (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_SHIFT                (0x00000004U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_DAC_CALIB_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MASK              (0x00000020U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_SHIFT             (0x00000005U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_SW_CAP_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MASK              (0x000003C0U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_RTRIM_0_3_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MASK        (0x00000C00U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_SHIFT       (0x0000000AU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GMCOMP_CTRL_0_1_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MASK           (0x00003000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM3_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MASK           (0x0000C000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_GM2_CTRL_0_1_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MASK              (0x00080000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_SHIFT             (0x00000013U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_REFSYS_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MASK              (0x00100000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_SHIFT             (0x00000014U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MASK                 (0x00200000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_SHIFT                (0x00000015U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_0P9G_MODE_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MASK             (0x00400000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_SHIFT            (0x00000016U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CHOP_HALF_CLK_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MASK           (0x00800000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_SHIFT          (0x00000017U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_CLK_3P6OR1P8G_CTRL_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MASK                   (0x01000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_IFA_CAL_CLK_EN_SHIFT                  (0x00000018U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_IFA_CAL_CLK_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_IFA_CAL_CLK_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_EN_MASK                     (0x02000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_EN_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_IBIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MASK            (0x04000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_SHIFT           (0x0000001AU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_100M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MASK            (0x08000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_SHIFT           (0x0000001BU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_200M_CLK_EDGE_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MASK              (0x10000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_SHIFT             (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EDGE_SEL_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EN_MASK                    (0x20000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EN_SHIFT                   (0x0000001DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_TX_DAC_CLK_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MASK             (0xC0000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_SHIFT            (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_ADC_SAT_DET_CTRL_0_1_MAX              (0x00000003U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_3_RESETVAL                              (0x00000000U)

/* RX4_ADC_I_CTRL_2 */

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_GM2_IBLEED_CTRL_2_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MASK             (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_RES_FF2_CTRL_0_2_MAX              (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MASK       (0x000000F0U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MASK       (0x00000F00U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_SHIFT      (0x00000008U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC11_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MASK       (0x0000F000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MASK       (0x000F0000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_SHIFT      (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC12_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MASK       (0x00F00000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_P_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MASK       (0x0F000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_SHIFT      (0x00000018U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC13_N_CALIB_CTRL_0_3_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MASK        (0xF0000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_SHIFT       (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_ADC_DAC2_P_CALIB_CTRL_0_3_MAX         (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_2_RESETVAL                              (0x00000000U)

/* RX4_ADC_I_CTRL_1 */

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MASK               (0x00000007U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC11_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MASK               (0x00000038U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC12_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MASK               (0x000001C0U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_SHIFT              (0x00000006U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC13_CTRL_0_2_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MASK                (0x00000E00U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_SHIFT               (0x00000009U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CTRL_0_2_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_IBIAS_SEL_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MASK             (0x00002000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_RST_BUF_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC2_CHOP_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MASK             (0x00008000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_SHIFT            (0x0000000FU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_DAC_CHOP_DISABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MASK        (0x00070000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GMCOMP_IBIAS_CTRL_0_2_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MASK           (0x00380000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_SHIFT          (0x00000013U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MASK              (0x00400000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_SHIFT             (0x00000016U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF3_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MASK          (0x03800000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM3_IBLEED_CTRL_0_2_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MASK           (0x1C000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_SHIFT          (0x0000001AU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBIAS_CTRL_0_2_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MASK              (0x20000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_SHIFT             (0x0000001DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_RES_FF2_DISABLE_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MASK          (0xC0000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_ADC_GM2_IBLEED_CTRL_0_1_MAX           (0x00000003U)

#define CSL_AR_RFANACIO_RX4_ADC_I_CTRL_1_RESETVAL                              (0x00000000U)

/* RX4_ADC_I_ANA_LDO_CTRL */

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MASK      (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_VOUT_CTRL_MAX       (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_SC_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_ADC_ANA_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_RESERVED1_MASK                  (0xFFFF0000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_RESERVED1_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_RESERVED1_MAX                   (0x0000FFFFU)

#define CSL_AR_RFANACIO_RX4_ADC_I_ANA_LDO_CTRL_RESETVAL                        (0x00000000U)

/* RX4_I_IFA_TEST_CTRL */

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VOUT_SENSE_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MASK        (0x00000002U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_SHIFT       (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VIN_SENSE_TEST_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MASK             (0x00000004U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_SHIFT            (0x00000002U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_VSSA_TEST_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MASK          (0x00000008U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_SHIFT         (0x00000003U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_LDO_ITEST_12P5UA_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_INPUT_TEST_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_INPUT_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_INPUT_TEST_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED0_MASK                     (0x00000020U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED0_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED0_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MASK         (0x00000040U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_SHIFT        (0x00000006U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_VMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MASK         (0x00000080U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_SHIFT        (0x00000007U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_OUTPUT_TEST_IMODE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED1_MASK                     (0x00000F00U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED1_SHIFT                    (0x00000008U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED1_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_REFGEN_VCM_TEST_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MASK    (0x00002000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_SHIFT   (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_VCM_OUT_TEST_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MASK      (0x00004000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_SHIFT     (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_VCM_OUT_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_VCM_OUT_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MASK    (0x00010000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_MAIN_AMP_ITEST_12P5UA_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_FB_AMP_ITEST_12P5UA_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MASK       (0x00040000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_SHIFT      (0x00000012U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA2_OPAMP_ITEST_12P5UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_BIAS_GEN_ITEST_10UA_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MASK         (0x00100000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_SHIFT        (0x00000014U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RSIG_TRIM_VSENSE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RGM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_R1_TRIM_VSENSE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MASK          (0x00800000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_SHIFT         (0x00000017U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA1_RCM_TRIM_VSENSE_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MASK (0x01000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_IFA_MODE_CTRL_DISABLE_IFA_HPFB_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED2_MASK                     (0xFE000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED2_SHIFT                    (0x00000019U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESERVED2_MAX                      (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_TEST_CTRL_RESETVAL                           (0x00000000U)

/* RX4_I_IFA_CTRL_5 */

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MASK           (0xF0000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_SHIFT          (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5_IFA_MAIN_AMP_CMFB_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_5_RESETVAL                              (0x00000000U)

/* RX4_I_IFA_CTRL_4 */

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MASK                   (0x0000003FU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_CTRIM_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MASK                   (0x00000FC0U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_CTRIM_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_CTRIM_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_CTRIM_MAX                    (0x0000003FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_SEL_MASK                     (0x00007000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_SEL_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFA_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_SEL_MASK                     (0x00038000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_SEL_SHIFT                    (0x0000000FU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_SEL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPFB_SEL_MAX                      (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_R1B_RTRIM_MASK                    (0x00FC0000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_R1B_RTRIM_SHIFT                   (0x00000012U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_R1B_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_R1B_RTRIM_MAX                     (0x0000003FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MASK          (0x07000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_MAIN_AMP_STG1_IBIAS_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MASK           (0x18000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_SHIFT          (0x0000001BU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_FBB_AMP_STG1_IBIAS_MAX            (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MASK       (0x20000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_SHIFT      (0x0000001DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_MASK_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_IFA_HPF_FAST_INIT_R1B_CODE_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_4_RESETVAL                              (0x00000000U)

/* RX4_I_IFA_CTRL_3 */

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGA_RTRIM_MASK                    (0x0000007FU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGA_RTRIM_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGA_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGA_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGB_RTRIM_MASK                    (0x00003F80U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGB_RTRIM_SHIFT                   (0x00000007U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGB_RTRIM_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_RGB_RTRIM_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF_CTRL_MASK                     (0x003FC000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF_CTRL_SHIFT                    (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF_CTRL_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF_CTRL_MAX                      (0x000000FFU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MASK               (0x00C00000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_SHIFT              (0x00000016U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_ADC_LPF2_GAIN_CTRL_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MASK                 (0x06000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_SHIFT                (0x00000019U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_OUT_RES_CTRL_MAX                  (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_REDERVED0_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_REDERVED0_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_REDERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_REDERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MASK                 (0x10000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_SHIFT                (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_IN_RES_CTRL2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_IFA_CAL_CLK_SYNCH_MODE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_RESERVED0_MASK                        (0xC0000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_RESERVED0_SHIFT                       (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_RESERVED0_MAX                         (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_3_RESETVAL                              (0x00000000U)

/* RX4_I_IFA_CTRL_2 */

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_EN_MASK                    (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_EN_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MASK              (0x0000007EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_SHIFT             (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_MAG_CTRL_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_OFFDAC_SIGN_CTRL_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_EN_MASK                    (0x00000100U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_EN_SHIFT                   (0x00000008U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_LOW_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MASK                  (0x0000FC00U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_CTRL_SHIFT                 (0x0000000AU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_CTRL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CALDAC_CTRL_MAX                   (0x0000003FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_EN_MASK                   (0x00010000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_EN_SHIFT                  (0x00000010U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MASK           (0x00020000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_SHIFT          (0x00000011U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_HI_BIAS_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MASK                 (0x00040000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_SHIFT                (0x00000012U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_RSTZ_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MASK       (0x00080000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CLIPDET_VOUT_LOGIC_SEL_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_EN_MUX_SEL_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_EN_MUX_SEL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_EN_MUX_SEL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_CAL_EN_VDD_DIG_FILT_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MASK           (0x00400000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_SHIFT          (0x00000016U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_MASK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MASK       (0xFF800000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_SHIFT      (0x00000017U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_IFA_HPF_FAST_INIT_R1A_TRIM_MAX        (0x000001FFU)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_2_RESETVAL                              (0x00000000U)

/* RX4_I_IFA_REFGEN_LDO_CTRL */

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MASK       (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_SHIFT      (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_VOUT_CTRL_MAX        (0x0000000FU)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MASK      (0x00000020U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_SHIFT     (0x00000005U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_LOW_BW_ENZ_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MASK     (0x00000040U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_SHIFT    (0x00000006U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SHRT_CKT_EN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MASK       (0x00000080U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_SHIFT      (0x00000007U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BYPASS_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MASK         (0x00000700U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_BW_CTRL_MAX          (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_PMOS_PULLDWN_EN_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MASK      (0x0000E000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_SHIFT     (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_LDO_TLOAD_CTRL_MAX       (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MASK           (0x00010000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_SHIFT          (0x00000010U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_EN_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MASK (0x000E0000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBB_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MASK (0x00700000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_SHIFT (0x00000014U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_CTRL_MAX  (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MASK (0x03800000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_SHIFT (0x00000017U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VCM_IFA_FBA_CTRL_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MASK (0x1C000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_SHIFT (0x0000001AU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_HI_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MASK (0xE0000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_SHIFT (0x0000001DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_IFA_REFGEN_VREF_CLIPDET_LO_MAX (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_REFGEN_LDO_CTRL_RESETVAL                     (0x00000000U)

/* RX4_I_IFA_CTRL_1 */

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MASK                  (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_EN_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_STARTUP_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MASK          (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_IBIAS_CTRL_MAX           (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED0_MASK                        (0x00000020U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED0_SHIFT                       (0x00000005U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MASK        (0x000000C0U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_SHIFT       (0x00000006U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_OUT_STG_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_CAP_MIL_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_MAIN_AMP_RES_DEGEN_CTRL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MASK                   (0x00001000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_EN_SHIFT                  (0x0000000CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MASK              (0x00002000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_SHIFT             (0x0000000DU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MASK           (0x0001C000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED1_MASK                        (0x00020000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED1_SHIFT                       (0x00000011U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED1_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED1_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MASK         (0x000C0000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_SHIFT        (0x00000012U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MASK       (0x00300000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBA_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MASK                   (0x00400000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_EN_SHIFT                  (0x00000016U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MASK              (0x00800000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_SHIFT             (0x00000017U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_STARTUP_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MASK           (0x07000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_SHIFT          (0x00000018U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_IBIAS_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED2_MASK                        (0x08000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED2_SHIFT                       (0x0000001BU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED2_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESERVED2_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MASK         (0x30000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_SHIFT        (0x0000001CU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_OUT_STG_CTRL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MASK       (0xC0000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_SHIFT      (0x0000001EU)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_IFA_FBB_AMP_RES_DEGEN_CTRL_MAX        (0x00000003U)

#define CSL_AR_RFANACIO_RX4_I_IFA_CTRL_1_RESETVAL                              (0x00000000U)

/* TX_TOP_BIST_LNA_CTRL */

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MASK           (0x0000001FU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_SHIFT          (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MAX            (0x0000001FU)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MASK          (0x000003E0U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_SHIFT         (0x00000005U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MAX           (0x0000001FU)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MASK   (0x00000400U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_SHIFT  (0x0000000AU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MASK     (0x0001F800U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_SHIFT    (0x0000000BU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MAX      (0x0000003FU)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MASK     (0x00020000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_SHIFT    (0x00000011U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MASK  (0x00040000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_SHIFT (0x00000012U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MASK    (0x00380000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_SHIFT   (0x00000013U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MAX     (0x00000007U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MASK  (0x00C00000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_SHIFT (0x00000016U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MAX   (0x00000003U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MASK         (0x01000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_SHIFT        (0x00000018U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_RESERVED0_MASK                    (0xFE000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_RESERVED0_SHIFT                   (0x00000019U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_RESERVED0_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_TX_TOP_BIST_LNA_CTRL_RESETVAL                          (0x00000000U)

/* TX_TOP_BIST_EN_BYPASS */

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA1_BIST_EN_BYPASS_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA1_BIST_EN_BYPASS_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA1_BIST_EN_BYPASS_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA1_BIST_EN_BYPASS_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA1_BIST_EN_BYPASS_MASK      (0x00000002U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA1_BIST_EN_BYPASS_SHIFT     (0x00000001U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA1_BIST_EN_BYPASS_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA1_BIST_EN_BYPASS_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE1_BIST_EN_BYPASS_MASK   (0x00000004U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE1_BIST_EN_BYPASS_SHIFT  (0x00000002U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE1_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE1_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO1_BIST_EN_BYPASS_MASK   (0x00000008U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO1_BIST_EN_BYPASS_SHIFT  (0x00000003U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO1_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO1_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO1_BIST_EN_BYPASS_MASK    (0x00000010U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO1_BIST_EN_BYPASS_SHIFT   (0x00000004U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO1_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO1_BIST_EN_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO1_BIST_EN_BYPASS_MASK   (0x00000020U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO1_BIST_EN_BYPASS_SHIFT  (0x00000005U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO1_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO1_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA2_BIST_EN_BYPASS_MASK       (0x00000040U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA2_BIST_EN_BYPASS_SHIFT      (0x00000006U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA2_BIST_EN_BYPASS_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA2_BIST_EN_BYPASS_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA2_BIST_EN_BYPASS_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA2_BIST_EN_BYPASS_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA2_BIST_EN_BYPASS_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA2_BIST_EN_BYPASS_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE2_BIST_EN_BYPASS_MASK   (0x00000100U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE2_BIST_EN_BYPASS_SHIFT  (0x00000008U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE2_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE2_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO2_BIST_EN_BYPASS_MASK   (0x00000200U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO2_BIST_EN_BYPASS_SHIFT  (0x00000009U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO2_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO2_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO2_BIST_EN_BYPASS_MASK    (0x00000400U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO2_BIST_EN_BYPASS_SHIFT   (0x0000000AU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO2_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO2_BIST_EN_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO2_BIST_EN_BYPASS_MASK   (0x00000800U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO2_BIST_EN_BYPASS_SHIFT  (0x0000000BU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO2_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO2_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA3_BIST_EN_BYPASS_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA3_BIST_EN_BYPASS_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA3_BIST_EN_BYPASS_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA3_BIST_EN_BYPASS_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA3_BIST_EN_BYPASS_MASK      (0x00002000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA3_BIST_EN_BYPASS_SHIFT     (0x0000000DU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA3_BIST_EN_BYPASS_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPA3_BIST_EN_BYPASS_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE3_BIST_EN_BYPASS_MASK   (0x00004000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE3_BIST_EN_BYPASS_SHIFT  (0x0000000EU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE3_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TSENSE3_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO3_BIST_EN_BYPASS_MASK   (0x00008000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO3_BIST_EN_BYPASS_SHIFT  (0x0000000FU)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO3_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PPALDO3_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO3_BIST_EN_BYPASS_MASK    (0x00010000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO3_BIST_EN_BYPASS_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO3_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PSLDO3_BIST_EN_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO3_BIST_EN_BYPASS_MASK   (0x00020000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO3_BIST_EN_BYPASS_SHIFT  (0x00000011U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO3_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_DACLDO3_BIST_EN_BYPASS_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA_LDO_BIST_EN_BYPASS_MASK    (0x00040000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA_LDO_BIST_EN_BYPASS_SHIFT   (0x00000012U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA_LDO_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_PA_LDO_BIST_EN_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TOP_LB_BIST_EN_BYPASS_MASK    (0x00080000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TOP_LB_BIST_EN_BYPASS_SHIFT   (0x00000013U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TOP_LB_BIST_EN_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_TX_TOP_LB_BIST_EN_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_RESERVED0_MASK                   (0xFFF00000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_RESERVED0_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_RESERVED0_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_RESERVED0_MAX                    (0x00000FFFU)

#define CSL_AR_RFANACIO_TX_TOP_BIST_EN_BYPASS_RESETVAL                         (0x00000000U)

/* TX_TOP_EN */

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA1_MASK                           (0x00000001U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA1_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA2_MASK                           (0x00000002U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA2_SHIFT                          (0x00000001U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA3_MASK                           (0x00000004U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA3_SHIFT                          (0x00000002U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA3_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA3_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA4_MASK                           (0x00000008U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA4_SHIFT                          (0x00000003U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA4_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_TOP_EN_PA4_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_BIST_LNATOP_STATIC_ENABLE_MASK            (0x00000010U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_BIST_LNATOP_STATIC_ENABLE_SHIFT           (0x00000004U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_BIST_LNATOP_STATIC_ENABLE_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_BIST_LNATOP_STATIC_ENABLE_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA1_STATIC_ENABLE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA1_STATIC_ENABLE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA1_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA1_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA2_STATIC_ENABLE_MASK             (0x00000040U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA2_STATIC_ENABLE_SHIFT            (0x00000006U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA2_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX1_BIST_LNA2_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA1_STATIC_ENABLE_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA1_STATIC_ENABLE_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA1_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA1_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA2_STATIC_ENABLE_MASK             (0x00000100U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA2_STATIC_ENABLE_SHIFT            (0x00000008U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA2_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX2_BIST_LNA2_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA1_STATIC_ENABLE_MASK             (0x00000200U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA1_STATIC_ENABLE_SHIFT            (0x00000009U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA1_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA1_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA2_STATIC_ENABLE_MASK             (0x00000400U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA2_STATIC_ENABLE_SHIFT            (0x0000000AU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA2_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX3_BIST_LNA2_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA1_STATIC_ENABLE_MASK             (0x00000800U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA1_STATIC_ENABLE_SHIFT            (0x0000000BU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA1_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA1_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA2_STATIC_ENABLE_MASK             (0x00001000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA2_STATIC_ENABLE_SHIFT            (0x0000000CU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA2_STATIC_ENABLE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX4_BIST_LNA2_STATIC_ENABLE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CLK_SEL_MASK                   (0x00002000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CLK_SEL_SHIFT                  (0x0000000DU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CLK_SEL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CLK_SEL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_BYPASS_CLK_GATE_MASK           (0x00004000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_BYPASS_CLK_GATE_SHIFT          (0x0000000EU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_BYPASS_CLK_GATE_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_BYPASS_CLK_GATE_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CTRL_MASK                      (0x001F8000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CTRL_SHIFT                     (0x0000000FU)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CTRL_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_DAC_RETIME_CTRL_MAX                       (0x0000003FU)

#define CSL_AR_RFANACIO_TX_TOP_EN_TX_10UA_IMIRROR_ENABLE_MASK                  (0x00200000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_10UA_IMIRROR_ENABLE_SHIFT                 (0x00000015U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_10UA_IMIRROR_ENABLE_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_TX_10UA_IMIRROR_ENABLE_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_EN_BIAS_TX4_MASK                          (0x00400000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_EN_BIAS_TX4_SHIFT                         (0x00000016U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_EN_BIAS_TX4_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_EN_BIAS_TX4_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_RAPID_EN_BIAS_TX4_MASK                    (0x00800000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_RAPID_EN_BIAS_TX4_SHIFT                   (0x00000017U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_RAPID_EN_BIAS_TX4_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_RAPID_EN_BIAS_TX4_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_OUTBUF_EN_BIAS_TX4_MASK                   (0x01000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_OUTBUF_EN_BIAS_TX4_SHIFT                  (0x00000018U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_OUTBUF_EN_BIAS_TX4_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_OUTBUF_EN_BIAS_TX4_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEI_EN_BIAS_TX4_MASK                (0x02000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEI_EN_BIAS_TX4_SHIFT               (0x00000019U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEI_EN_BIAS_TX4_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEI_EN_BIAS_TX4_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEQ_EN_BIAS_TX4_MASK                (0x04000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEQ_EN_BIAS_TX4_SHIFT               (0x0000001AU)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEQ_EN_BIAS_TX4_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_VMLOGATEQ_EN_BIAS_TX4_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_I_EN_BIAS_TX4_MASK               (0x08000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_I_EN_BIAS_TX4_SHIFT              (0x0000001BU)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_I_EN_BIAS_TX4_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_I_EN_BIAS_TX4_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_Q_EN_BIAS_TX4_MASK               (0x10000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_Q_EN_BIAS_TX4_SHIFT              (0x0000001CU)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_Q_EN_BIAS_TX4_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_IQG_STG1_Q_EN_BIAS_TX4_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_RESERVED1_MASK                               (0x20000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_RESERVED1_SHIFT                              (0x0000001DU)
#define CSL_AR_RFANACIO_TX_TOP_EN_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_RESERVED1_MAX                                (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_EN_LODIST_TX4_AMP_MASK                       (0x40000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_EN_LODIST_TX4_AMP_SHIFT                      (0x0000001EU)
#define CSL_AR_RFANACIO_TX_TOP_EN_EN_LODIST_TX4_AMP_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_EN_LODIST_TX4_AMP_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_PS_SEL_RAPID_ENABLE_TX4_MASK                 (0x80000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_SEL_RAPID_ENABLE_TX4_SHIFT                (0x0000001FU)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_SEL_RAPID_ENABLE_TX4_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_EN_PS_SEL_RAPID_ENABLE_TX4_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_EN_RESETVAL                                     (0x00000000U)

/* TX_TOP_TESTMUX_CTRL */

#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED1_MASK                     (0x0000000FU)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED1_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED1_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_TX_BIST_BUFF_ATEST_ENABLE_MASK     (0x00000010U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_TX_BIST_BUFF_ATEST_ENABLE_SHIFT    (0x00000004U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_TX_BIST_BUFF_ATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_TX_BIST_BUFF_ATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED0_MASK                     (0xFFFFFFE0U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED0_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESERVED0_MAX                      (0x07FFFFFFU)

#define CSL_AR_RFANACIO_TX_TOP_TESTMUX_CTRL_RESETVAL                           (0x00000000U)

/* TX_CONFIG_LOOPBACK */

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_STG2_BUF_BIAS_MASK       (0x00000001U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_STG2_BUF_BIAS_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_STG2_BUF_BIAS_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_STG2_BUF_BIAS_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_STG2_BUF_BIAS_MASK          (0x0000007EU)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_STG2_BUF_BIAS_SHIFT         (0x00000001U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_STG2_BUF_BIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_STG2_BUF_BIAS_MAX           (0x0000003FU)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED0_MASK                      (0x00000080U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED0_SHIFT                     (0x00000007U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED0_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_QBUF_BIAS_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_QBUF_BIAS_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_QBUF_BIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_EN_LB_QPSK_QBUF_BIAS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_QBUF_BIAS_MASK              (0x00007E00U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_QBUF_BIAS_SHIFT             (0x00000009U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_QBUF_BIAS_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_LB_QPSK_QBUF_BIAS_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED1_MASK                      (0x00008000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED1_SHIFT                     (0x0000000FU)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED1_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED2_MASK                      (0x00010000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED2_SHIFT                     (0x00000010U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED2_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED2_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED3_MASK                      (0x007E0000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED3_SHIFT                     (0x00000011U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED3_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED3_MAX                       (0x0000003FU)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED4_MASK                      (0x00800000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED4_SHIFT                     (0x00000017U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED4_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED4_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED5_MASK                      (0x01000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED5_SHIFT                     (0x00000018U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED5_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED5_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED6_MASK                      (0x7E000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED6_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED6_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED6_MAX                       (0x0000003FU)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED7_MASK                      (0x80000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED7_SHIFT                     (0x0000001FU)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED7_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESERVED7_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_CONFIG_LOOPBACK_RESETVAL                            (0x00000000U)

/* TX1_DAC_CTRL */

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKDLY_MASK                           (0x00000007U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKDLY_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKINVEN_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKINVEN_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ILATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLKINV_MASK                              (0x00000010U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLKINV_SHIFT                             (0x00000004U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_ICAL_MASK                              (0x00000020U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_ICAL_SHIFT                             (0x00000005U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_ICAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_ICAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICALCLKSEL_MASK                           (0x000003C0U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICALCLKSEL_SHIFT                          (0x00000006U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_IDEC_MASK                              (0x00000400U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_IDEC_SHIFT                             (0x0000000AU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_IDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_IDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACI_RESETB_MASK                          (0x00000800U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACI_RESETB_SHIFT                         (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACI_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACI_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVI_DEC_MASK                          (0x00001000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVI_DEC_SHIFT                         (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVI_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVI_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLK_DEC_EN_MASK                          (0x00002000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLK_DEC_EN_SHIFT                         (0x0000000DU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ICLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ISIGN_EN_MASK                             (0x00004000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ISIGN_EN_SHIFT                            (0x0000000EU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ISIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_ISIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKDLY_MASK                           (0x00038000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKDLY_SHIFT                          (0x0000000FU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKINVEN_MASK                         (0x00040000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKINVEN_SHIFT                        (0x00000012U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QLATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLKINV_MASK                              (0x00080000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLKINV_SHIFT                             (0x00000013U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QCAL_MASK                              (0x00100000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QCAL_SHIFT                             (0x00000014U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QCAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QCAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCALCLKSEL_MASK                           (0x01E00000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCALCLKSEL_SHIFT                          (0x00000015U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QDEC_MASK                              (0x02000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QDEC_SHIFT                             (0x00000019U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_EN_QDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACQ_RESETB_MASK                          (0x04000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACQ_RESETB_SHIFT                         (0x0000001AU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACQ_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_DACQ_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVQ_DEC_MASK                          (0x08000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVQ_DEC_SHIFT                         (0x0000001BU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVQ_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_CLKDIVQ_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLK_DEC_EN_MASK                          (0x10000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLK_DEC_EN_SHIFT                         (0x0000001CU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QCLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QSIGN_EN_MASK                             (0x20000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QSIGN_EN_SHIFT                            (0x0000001DU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QSIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_QSIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_RESERVED_MASK                             (0xC0000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_RESERVED_SHIFT                            (0x0000001EU)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CTRL_RESERVED_MAX                              (0x00000003U)

#define CSL_AR_RFANACIO_TX1_DAC_CTRL_RESETVAL                                  (0x00000000U)

/* TX1_DAC_CONFIG */

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_EN_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_EN_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_EN_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_EN_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_CTRL_MASK                          (0x0000000EU)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_CTRL_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_CTRL_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_BIAS_CTRL_MAX                           (0x00000007U)

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_ICNTRL_MASK                         (0x00000FF0U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_ICNTRL_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_ICNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_ICNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_QCNTRL_MASK                         (0x000FF000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_QCNTRL_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_QCNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_DAC_QCNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_RESERVED_MASK                           (0xFFF00000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_RESERVED_SHIFT                          (0x00000014U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_RESERVED_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_RESERVED_MAX                            (0x00000FFFU)

#define CSL_AR_RFANACIO_TX1_DAC_CONFIG_RESETVAL                                (0x00000000U)

/* TX1_PA_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG1_MASK                 (0x0000003FU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG1_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG1_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG2_MASK                 (0x00000FC0U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG2_SHIFT                (0x00000006U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG2_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MASK                (0x0003F000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3L_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3R_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_RESERVED0_MASK                    (0x1F000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_RESERVED0_SHIFT                   (0x00000018U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_RESERVED0_MAX                     (0x0000001FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MASK                (0xE0000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_IBIAS_TEST_SHIFT               (0x0000001DU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_IBIAS_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX1_PA_BIASCTRL_REG2 */

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MASK                (0x0000003FU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4L_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MASK                (0x00000FC0U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4R_SHIFT               (0x00000006U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_RESERVED0_MASK                    (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_RESERVED0_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_RESERVED0_MAX                     (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX1_PA_BIASCTRL_REG2_RESETVAL                          (0x00000000U)

/* TX1_PA_EN */

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG1_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG1_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG1_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG2_MASK                          (0x00000002U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG2_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG2_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3L_MASK                         (0x00000004U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3L_SHIFT                        (0x00000002U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3R_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3R_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG3R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4L_MASK                         (0x00000010U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4L_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4R_MASK                         (0x00000020U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4R_SHIFT                        (0x00000005U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_STG4R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_BIAS_MASTER_EN_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_BIAS_MASTER_EN_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_BIAS_MASTER_EN_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_BIAS_MASTER_EN_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_CTRL_PATH_MASK                     (0x00000080U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_CTRL_PATH_SHIFT                    (0x00000007U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_CTRL_PATH_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_PA_ENABLE_CTRL_PATH_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED0_MASK                               (0x00000F00U)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED0_SHIFT                              (0x00000008U)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED0_MAX                                (0x0000000FU)

#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED1_MASK                               (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED1_SHIFT                              (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_EN_RESERVED1_MAX                                (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX1_PA_EN_RESETVAL                                     (0x00000000U)

/* TX1_PA_SPARES */

#define CSL_AR_RFANACIO_TX1_PA_SPARES_RESERVED0_MASK                           (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_TX1_PA_SPARES_RESERVED0_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_SPARES_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_SPARES_RESERVED0_MAX                            (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_TX1_PA_SPARES_RESETVAL                                 (0x00000000U)

/* TX1_PA_TESTMUX_CTRL */

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED1_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED1_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MASK         (0x00000002U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_SHIFT        (0x00000001U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MASK       (0x00000004U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_SHIFT      (0x00000002U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MASK       (0x00000008U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_SHIFT      (0x00000003U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MASK       (0x00000010U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MASK       (0x00000020U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MASK      (0x00000040U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_SHIFT     (0x00000006U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MASK         (0x00000100U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MASK       (0x00000200U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_SHIFT      (0x00000009U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MASK       (0x00000400U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_SHIFT      (0x0000000AU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MASK      (0x00001000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_SHIFT     (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MASK         (0x00002000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_SHIFT        (0x0000000DU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MASK       (0x00004000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_SHIFT      (0x0000000EU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MASK      (0x00010000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_SHIFT     (0x00000010U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MASK        (0x00040000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_SHIFT       (0x00000012U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MASK       (0x00100000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED0_MASK                     (0x01E00000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED0_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESERVED0_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_TX1_PA_TESTMUX_CTRL_RESETVAL                           (0x00000000U)

/* TX1_DAC_DIGLDO_CTRL */

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MASK           (0x0000003EU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MAX            (0x0000001FU)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MASK        (0x00000100U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MASK (0x00000200U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_SHIFT (0x00000009U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MASK       (0x00000800U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_SHIFT      (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MASK   (0x001FE000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_SHIFT  (0x0000000DU)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MAX    (0x000000FFU)

#define CSL_AR_RFANACIO_TX1_DAC_DIGLDO_CTRL_RESETVAL                           (0x00000000U)

/* TX1_DAC_ANALDO_CTRL */

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MASK           (0x0000001EU)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MASK (0x00010000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MASK (0x00020000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MASK      (0x00040000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_SHIFT     (0x00000012U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MASK        (0x00100000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_SHIFT       (0x00000014U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX1_DAC_ANALDO_CTRL_RESETVAL                           (0x00000000U)

/* TX1_IQGEN_BIAS_CTRL */

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MASK              (0x0000003FU)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MASK              (0x00000FC0U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX12_ROOT_AMP_MASK          (0x0003F000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX12_ROOT_AMP_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX12_ROOT_AMP_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX12_ROOT_AMP_MAX           (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX1_AMP_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX1_AMP_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX1_AMP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_LODIST_TX1_AMP_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_PA_LB_BUF_BIAS_MASK                (0x3F000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_PA_LB_BUF_BIAS_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_PA_LB_BUF_BIAS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_PA_LB_BUF_BIAS_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_EN_PA_LB_BUF_BIAS_MASK             (0x40000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_EN_PA_LB_BUF_BIAS_SHIFT            (0x0000001EU)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_EN_PA_LB_BUF_BIAS_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_EN_PA_LB_BUF_BIAS_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_RESERVED_MASK                      (0x80000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_RESERVED_SHIFT                     (0x0000001FU)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_RESERVED_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_RESERVED_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGEN_BIAS_CTRL_RESETVAL                           (0x00000000U)

/* TX1_IQGENLDO_CTRL */

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MASK         (0x00000001U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_SHIFT        (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MASK           (0x000000FEU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MAX            (0x0000007FU)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MASK     (0x00000100U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_SHIFT    (0x00000008U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MASK      (0x00000200U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_SHIFT     (0x00000009U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MASK     (0x00000800U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_SHIFT    (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MASK (0x00002000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED0_MASK                       (0x0000C000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED0_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED0_MAX                        (0x00000003U)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED1_MASK                       (0xFFFF0000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED1_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESERVED1_MAX                        (0x0000FFFFU)

#define CSL_AR_RFANACIO_TX1_IQGENLDO_CTRL_RESETVAL                             (0x00000000U)

/* TX1_CTRL_RC_FILT */

#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MASK           (0x03U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_SHIFT          (0x00U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_RESETVAL       (0x00U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MAX            (0x03U)

#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_RESERVED0_MASK                        (0x0CU)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_RESERVED0_SHIFT                       (0x02U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_RESERVED0_RESETVAL                    (0x00U)
#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_RESERVED0_MAX                         (0x03U)

#define CSL_AR_RFANACIO_TX1_CTRL_RC_FILT_RESETVAL                              (0x00U)

/* TX1_PS_TMUXCTRL */

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MASK              (0x0001U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_SHIFT             (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MASK              (0x0002U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_SHIFT             (0x0001U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MASK              (0x0004U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_SHIFT             (0x0002U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MASK              (0x0008U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_SHIFT             (0x0003U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MASK                 (0x0010U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_SHIFT                (0x0004U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MASK             (0x0020U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_SHIFT            (0x0005U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MASK             (0x0040U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_SHIFT            (0x0006U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MASK             (0x0080U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_SHIFT            (0x0007U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MASK             (0x0100U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_SHIFT            (0x0008U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MASK               (0x0200U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_SHIFT              (0x0009U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MASK               (0x0400U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_SHIFT              (0x000AU)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MASK            (0x0800U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_SHIFT           (0x000BU)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MASK         (0x1000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_SHIFT        (0x000CU)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_RESETVAL     (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MAX          (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MASK                 (0x2000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_SHIFT                (0x000DU)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_TMUXCTRL_RESETVAL                               (0x0000U)

/* TX1_PS_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MASK (0x0000003FU)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MAX (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MASK (0x00000FC0U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_SHIFT (0x00000006U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MASK (0x0003F000U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_PS_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX1_PS_SPARES */

#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_EN_MASK                         (0x0001U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_EN_SHIFT                        (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_EN_RESETVAL                     (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_EN_MAX                          (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_CTRL_MASK                       (0x00FEU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_CTRL_SHIFT                      (0x0001U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_CTRL_RESETVAL                   (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_IBIAS_PD_CTRL_MAX                        (0x007FU)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_ANATEST_ENABLE_MASK                 (0x0100U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_ANATEST_ENABLE_SHIFT                (0x0008U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_ANATEST_ENABLE_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_ANATEST_ENABLE_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_VIN_ANATEST_ENABLE_MASK                  (0x0200U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VIN_ANATEST_ENABLE_SHIFT                 (0x0009U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VIN_ANATEST_ENABLE_RESETVAL              (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VIN_ANATEST_ENABLE_MAX                   (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_STG1_ANATEST_ENABLE_MASK            (0x0400U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_STG1_ANATEST_ENABLE_SHIFT           (0x000AU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_STG1_ANATEST_ENABLE_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VOUT_STG1_ANATEST_ENABLE_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_VREF_ANATEST_ENABLE_MASK                 (0x0800U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VREF_ANATEST_ENABLE_SHIFT                (0x000BU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VREF_ANATEST_ENABLE_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_VREF_ANATEST_ENABLE_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_ITEST_100UA_ANATEST_ENABLE_MASK          (0x1000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_ITEST_100UA_ANATEST_ENABLE_SHIFT         (0x000CU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_ITEST_100UA_ANATEST_ENABLE_RESETVAL      (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_ITEST_100UA_ANATEST_ENABLE_MAX           (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_BIAS_BLOCK_CURRENT_BOOST_MASK            (0x2000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_BIAS_BLOCK_CURRENT_BOOST_SHIFT           (0x000DU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_BIAS_BLOCK_CURRENT_BOOST_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_BIAS_BLOCK_CURRENT_BOOST_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_RESERVED0_MASK                           (0xC000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_RESERVED0_SHIFT                          (0x000EU)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_RESERVED0_RESETVAL                       (0x0000U)
#define CSL_AR_RFANACIO_TX1_PS_SPARES_RESERVED0_MAX                            (0x0003U)

#define CSL_AR_RFANACIO_TX1_PS_SPARES_RESETVAL                                 (0x0000U)

/* TX1_BIST_LNA_CTRL */

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MASK              (0x0000001FU)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MASK             (0x000003E0U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MAX              (0x0000001FU)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MASK      (0x00000400U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_SHIFT     (0x0000000AU)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MASK        (0x0001F800U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_SHIFT       (0x0000000BU)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MAX         (0x0000003FU)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MASK        (0x00020000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_SHIFT       (0x00000011U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MASK     (0x00040000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_SHIFT    (0x00000012U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MASK       (0x00380000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MAX        (0x00000007U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MASK     (0x00C00000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_SHIFT    (0x00000016U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MAX      (0x00000003U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_RESERVED0_MASK                       (0xFE000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_RESERVED0_SHIFT                      (0x00000019U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_RESERVED0_MAX                        (0x0000007FU)

#define CSL_AR_RFANACIO_TX1_BIST_LNA_CTRL_RESETVAL                             (0x00000000U)

/* TX_PS_MODE_CTRL */

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_MODE_MASK                       (0x0001U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_MODE_SHIFT                      (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_MODE_RESETVAL                   (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_MODE_MAX                        (0x0001U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_QUADRANT_SEL_MASK               (0x0006U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_QUADRANT_SEL_SHIFT              (0x0001U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_QUADRANT_SEL_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX1_PS_QUADRANT_SEL_MAX                (0x0003U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_MODE_MASK                       (0x0008U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_MODE_SHIFT                      (0x0003U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_MODE_RESETVAL                   (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_MODE_MAX                        (0x0001U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_QUADRANT_SEL_MASK               (0x0030U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_QUADRANT_SEL_SHIFT              (0x0004U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_QUADRANT_SEL_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX2_PS_QUADRANT_SEL_MAX                (0x0003U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_MODE_MASK                       (0x0040U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_MODE_SHIFT                      (0x0006U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_MODE_RESETVAL                   (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_MODE_MAX                        (0x0001U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_QUADRANT_SEL_MASK               (0x0180U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_QUADRANT_SEL_SHIFT              (0x0007U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_QUADRANT_SEL_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX3_PS_QUADRANT_SEL_MAX                (0x0003U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_MODE_MASK                       (0x0200U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_MODE_SHIFT                      (0x0009U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_MODE_RESETVAL                   (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_MODE_MAX                        (0x0001U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_QUADRANT_SEL_MASK               (0x0C00U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_QUADRANT_SEL_SHIFT              (0x000AU)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_QUADRANT_SEL_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_TX4_PS_QUADRANT_SEL_MAX                (0x0003U)

#define CSL_AR_RFANACIO_TX_PS_MODE_CTRL_RESETVAL                               (0x0000U)

/* TX_PS_EN */

#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX1_MASK                           (0x00000001U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX1_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX1_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX1_MASK                     (0x00000002U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX1_SHIFT                    (0x00000001U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX1_MASK                    (0x00000004U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX1_SHIFT                   (0x00000002U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX1_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX1_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX1_MASK                 (0x00000008U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX1_SHIFT                (0x00000003U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX1_MASK                 (0x00000010U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX1_SHIFT                (0x00000004U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX1_MASK                (0x00000020U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX1_SHIFT               (0x00000005U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX1_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX1_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX1_MASK                (0x00000040U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX1_SHIFT               (0x00000006U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX1_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX1_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX12_ROOT_AMP_MASK                  (0x00000080U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX12_ROOT_AMP_SHIFT                 (0x00000007U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX12_ROOT_AMP_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX12_ROOT_AMP_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX1_AMP_MASK                        (0x00000100U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX1_AMP_SHIFT                       (0x00000008U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX1_AMP_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX1_AMP_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX1_MASK                  (0x00000200U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX1_SHIFT                 (0x00000009U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX1_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX2_MASK                           (0x00000400U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX2_SHIFT                          (0x0000000AU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX2_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX2_MASK                     (0x00000800U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX2_SHIFT                    (0x0000000BU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX2_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX2_MASK                    (0x00001000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX2_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX2_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX2_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX2_MASK                 (0x00002000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX2_SHIFT                (0x0000000DU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX2_MASK                 (0x00004000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX2_SHIFT                (0x0000000EU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX2_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX2_MASK                (0x00008000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX2_SHIFT               (0x0000000FU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX2_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX2_MASK                (0x00010000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX2_SHIFT               (0x00000010U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX2_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_RESERVED1_MASK                                (0x00020000U)
#define CSL_AR_RFANACIO_TX_PS_EN_RESERVED1_SHIFT                               (0x00000011U)
#define CSL_AR_RFANACIO_TX_PS_EN_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_RESERVED1_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX2_AMP_MASK                        (0x00040000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX2_AMP_SHIFT                       (0x00000012U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX2_AMP_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX2_AMP_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX2_MASK                  (0x00080000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX2_SHIFT                 (0x00000013U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX2_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX2_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX3_MASK                           (0x00100000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX3_SHIFT                          (0x00000014U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX3_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_EN_BIAS_TX3_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX3_MASK                     (0x00200000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX3_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX3_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_RAPID_EN_BIAS_TX3_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX3_MASK                    (0x00400000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX3_SHIFT                   (0x00000016U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX3_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_OUTBUF_EN_BIAS_TX3_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX3_MASK                 (0x00800000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX3_SHIFT                (0x00000017U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX3_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEI_EN_BIAS_TX3_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX3_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX3_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX3_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_VMLOGATEQ_EN_BIAS_TX3_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX3_MASK                (0x02000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX3_SHIFT               (0x00000019U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX3_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_I_EN_BIAS_TX3_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX3_MASK                (0x04000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX3_SHIFT               (0x0000001AU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX3_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_IQG_STG1_Q_EN_BIAS_TX3_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX34_ROOT_AMP_MASK                  (0x08000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX34_ROOT_AMP_SHIFT                 (0x0000001BU)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX34_ROOT_AMP_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX34_ROOT_AMP_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX3_AMP_MASK                        (0x10000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX3_AMP_SHIFT                       (0x0000001CU)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX3_AMP_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_EN_LODIST_TX3_AMP_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX3_MASK                  (0x20000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX3_SHIFT                 (0x0000001DU)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX3_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_PS_EN_PS_SEL_RAPID_ENABLE_TX3_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_PS_EN_RESETVAL                                      (0x00000000U)

/* TX2_BIST_LNA_CTRL */

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MASK              (0x0000001FU)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MASK             (0x000003E0U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MAX              (0x0000001FU)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MASK      (0x00000400U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_SHIFT     (0x0000000AU)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MASK        (0x0001F800U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_SHIFT       (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MAX         (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MASK        (0x00020000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_SHIFT       (0x00000011U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MASK     (0x00040000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_SHIFT    (0x00000012U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MASK       (0x00380000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MAX        (0x00000007U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MASK     (0x00C00000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_SHIFT    (0x00000016U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MAX      (0x00000003U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_RESERVED0_MASK                       (0xFE000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_RESERVED0_SHIFT                      (0x00000019U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_RESERVED0_MAX                        (0x0000007FU)

#define CSL_AR_RFANACIO_TX2_BIST_LNA_CTRL_RESETVAL                             (0x00000000U)

/* TX2_PS_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MASK (0x0000003FU)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MAX (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MASK (0x00000FC0U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_SHIFT (0x00000006U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MASK (0x0003F000U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PS_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX2_PS_TMUXCTRL */

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MASK              (0x0001U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_SHIFT             (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MASK              (0x0002U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_SHIFT             (0x0001U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MASK              (0x0004U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_SHIFT             (0x0002U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MASK              (0x0008U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_SHIFT             (0x0003U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MASK                 (0x0010U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_SHIFT                (0x0004U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MASK             (0x0020U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_SHIFT            (0x0005U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MASK             (0x0040U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_SHIFT            (0x0006U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MASK             (0x0080U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_SHIFT            (0x0007U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MASK             (0x0100U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_SHIFT            (0x0008U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MASK               (0x0200U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_SHIFT              (0x0009U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MASK               (0x0400U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_SHIFT              (0x000AU)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MASK            (0x0800U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_SHIFT           (0x000BU)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MASK         (0x1000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_SHIFT        (0x000CU)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_RESETVAL     (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MAX          (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MASK                 (0x2000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_SHIFT                (0x000DU)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX2_PS_TMUXCTRL_RESETVAL                               (0x0000U)

/* TX2_CTRL_RC_FILT */

#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MASK           (0x03U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_SHIFT          (0x00U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_RESETVAL       (0x00U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MAX            (0x03U)

#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_RESERVED0_MASK                        (0x0CU)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_RESERVED0_SHIFT                       (0x02U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_RESERVED0_RESETVAL                    (0x00U)
#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_RESERVED0_MAX                         (0x03U)

#define CSL_AR_RFANACIO_TX2_CTRL_RC_FILT_RESETVAL                              (0x00U)

/* TX2_IQGENLDO_CTRL */

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MASK         (0x00000001U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_SHIFT        (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MASK           (0x000000FEU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MAX            (0x0000007FU)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MASK     (0x00000100U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_SHIFT    (0x00000008U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MASK      (0x00000200U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_SHIFT     (0x00000009U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MASK     (0x00000800U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_SHIFT    (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MASK (0x00002000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED0_MASK                       (0x0000C000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED0_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED0_MAX                        (0x00000003U)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED1_MASK                       (0xFFFF0000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED1_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESERVED1_MAX                        (0x0000FFFFU)

#define CSL_AR_RFANACIO_TX2_IQGENLDO_CTRL_RESETVAL                             (0x00000000U)

/* TX2_IQGEN_BIAS_CTRL */

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MASK              (0x0000003FU)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MASK              (0x00000FC0U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED0_MASK                     (0x0003F000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED0_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED0_MAX                      (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LODIST_TX2_AMP_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LODIST_TX2_AMP_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LODIST_TX2_AMP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LODIST_TX2_AMP_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LB_QPSK_IBUF_BIAS_MASK             (0x3F000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LB_QPSK_IBUF_BIAS_SHIFT            (0x00000018U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LB_QPSK_IBUF_BIAS_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_LB_QPSK_IBUF_BIAS_MAX              (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_EN_LB_QPSK_IBUF_BIAS_MASK          (0x40000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_EN_LB_QPSK_IBUF_BIAS_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_EN_LB_QPSK_IBUF_BIAS_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_EN_LB_QPSK_IBUF_BIAS_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED1_MASK                     (0x80000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED1_SHIFT                    (0x0000001FU)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESERVED1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_IQGEN_BIAS_CTRL_RESETVAL                           (0x00000000U)

/* TX2_DAC_ANALDO_CTRL */

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MASK           (0x0000001EU)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MASK (0x00010000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MASK (0x00020000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MASK      (0x00040000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_SHIFT     (0x00000012U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MASK        (0x00100000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_SHIFT       (0x00000014U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_ANALDO_CTRL_RESETVAL                           (0x00000000U)

/* TX2_DAC_DIGLDO_CTRL */

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MASK           (0x0000003EU)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MAX            (0x0000001FU)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MASK        (0x00000100U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MASK (0x00000200U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_SHIFT (0x00000009U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MASK       (0x00000800U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_SHIFT      (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MASK   (0x001FE000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_SHIFT  (0x0000000DU)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MAX    (0x000000FFU)

#define CSL_AR_RFANACIO_TX2_DAC_DIGLDO_CTRL_RESETVAL                           (0x00000000U)

/* TX2_PA_TESTMUX_CTRL */

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED1_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED1_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MASK         (0x00000002U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_SHIFT        (0x00000001U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MASK       (0x00000004U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_SHIFT      (0x00000002U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MASK       (0x00000008U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_SHIFT      (0x00000003U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MASK       (0x00000010U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MASK       (0x00000020U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MASK      (0x00000040U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_SHIFT     (0x00000006U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MASK         (0x00000100U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MASK       (0x00000200U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_SHIFT      (0x00000009U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MASK       (0x00000400U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_SHIFT      (0x0000000AU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MASK      (0x00001000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_SHIFT     (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MASK         (0x00002000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_SHIFT        (0x0000000DU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MASK       (0x00004000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_SHIFT      (0x0000000EU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MASK      (0x00010000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_SHIFT     (0x00000010U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MASK        (0x00040000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_SHIFT       (0x00000012U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MASK       (0x00100000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED0_MASK                     (0x01E00000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED0_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESERVED0_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_TX2_PA_TESTMUX_CTRL_RESETVAL                           (0x00000000U)

/* TX2_PA_SPARES */

#define CSL_AR_RFANACIO_TX2_PA_SPARES_RESERVED0_MASK                           (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_TX2_PA_SPARES_RESERVED0_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_SPARES_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_SPARES_RESERVED0_MAX                            (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_TX2_PA_SPARES_RESETVAL                                 (0x00000000U)

/* TX2_PA_EN */

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG1_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG1_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG1_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG2_MASK                          (0x00000002U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG2_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG2_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3L_MASK                         (0x00000004U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3L_SHIFT                        (0x00000002U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3R_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3R_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG3R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4L_MASK                         (0x00000010U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4L_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4R_MASK                         (0x00000020U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4R_SHIFT                        (0x00000005U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_STG4R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_BIAS_MASTER_EN_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_BIAS_MASTER_EN_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_BIAS_MASTER_EN_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_BIAS_MASTER_EN_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_CTRL_PATH_MASK                     (0x00000080U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_CTRL_PATH_SHIFT                    (0x00000007U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_CTRL_PATH_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_PA_ENABLE_CTRL_PATH_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED0_MASK                               (0x00000F00U)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED0_SHIFT                              (0x00000008U)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED0_MAX                                (0x0000000FU)

#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED1_MASK                               (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED1_SHIFT                              (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_EN_RESERVED1_MAX                                (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX2_PA_EN_RESETVAL                                     (0x00000000U)

/* TX2_PA_BIASCTRL_REG2 */

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MASK                (0x0000003FU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4L_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MASK                (0x00000FC0U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4R_SHIFT               (0x00000006U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_RESERVED0_MASK                    (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_RESERVED0_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_RESERVED0_MAX                     (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG2_RESETVAL                          (0x00000000U)

/* TX2_PA_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG1_MASK                 (0x0000003FU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG1_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG1_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG2_MASK                 (0x00000FC0U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG2_SHIFT                (0x00000006U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG2_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MASK                (0x0003F000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3L_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3R_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_RESERVED0_MASK                    (0x1F000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_RESERVED0_SHIFT                   (0x00000018U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_RESERVED0_MAX                     (0x0000001FU)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MASK                (0xE0000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_IBIAS_TEST_SHIFT               (0x0000001DU)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_IBIAS_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_TX2_PA_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX2_DAC_CONFIG */

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_EN_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_EN_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_EN_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_EN_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_CTRL_MASK                          (0x0000000EU)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_CTRL_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_CTRL_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_BIAS_CTRL_MAX                           (0x00000007U)

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_ICNTRL_MASK                         (0x00000FF0U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_ICNTRL_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_ICNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_ICNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_QCNTRL_MASK                         (0x000FF000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_QCNTRL_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_QCNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_DAC_QCNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_RESERVED_MASK                           (0xFFF00000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_RESERVED_SHIFT                          (0x00000014U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_RESERVED_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_RESERVED_MAX                            (0x00000FFFU)

#define CSL_AR_RFANACIO_TX2_DAC_CONFIG_RESETVAL                                (0x00000000U)

/* TX2_DAC_CTRL */

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKDLY_MASK                           (0x00000007U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKDLY_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKINVEN_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKINVEN_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ILATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLKINV_MASK                              (0x00000010U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLKINV_SHIFT                             (0x00000004U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_ICAL_MASK                              (0x00000020U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_ICAL_SHIFT                             (0x00000005U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_ICAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_ICAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICALCLKSEL_MASK                           (0x000003C0U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICALCLKSEL_SHIFT                          (0x00000006U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_IDEC_MASK                              (0x00000400U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_IDEC_SHIFT                             (0x0000000AU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_IDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_IDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACI_RESETB_MASK                          (0x00000800U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACI_RESETB_SHIFT                         (0x0000000BU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACI_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACI_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVI_DEC_MASK                          (0x00001000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVI_DEC_SHIFT                         (0x0000000CU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVI_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVI_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLK_DEC_EN_MASK                          (0x00002000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLK_DEC_EN_SHIFT                         (0x0000000DU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ICLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ISIGN_EN_MASK                             (0x00004000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ISIGN_EN_SHIFT                            (0x0000000EU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ISIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_ISIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKDLY_MASK                           (0x00038000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKDLY_SHIFT                          (0x0000000FU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKINVEN_MASK                         (0x00040000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKINVEN_SHIFT                        (0x00000012U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QLATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLKINV_MASK                              (0x00080000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLKINV_SHIFT                             (0x00000013U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QCAL_MASK                              (0x00100000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QCAL_SHIFT                             (0x00000014U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QCAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QCAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCALCLKSEL_MASK                           (0x01E00000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCALCLKSEL_SHIFT                          (0x00000015U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QDEC_MASK                              (0x02000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QDEC_SHIFT                             (0x00000019U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_EN_QDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACQ_RESETB_MASK                          (0x04000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACQ_RESETB_SHIFT                         (0x0000001AU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACQ_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_DACQ_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVQ_DEC_MASK                          (0x08000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVQ_DEC_SHIFT                         (0x0000001BU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVQ_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_CLKDIVQ_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLK_DEC_EN_MASK                          (0x10000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLK_DEC_EN_SHIFT                         (0x0000001CU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QCLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QSIGN_EN_MASK                             (0x20000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QSIGN_EN_SHIFT                            (0x0000001DU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QSIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_QSIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_RESERVED_MASK                             (0xC0000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_RESERVED_SHIFT                            (0x0000001EU)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX2_DAC_CTRL_RESERVED_MAX                              (0x00000003U)

#define CSL_AR_RFANACIO_TX2_DAC_CTRL_RESETVAL                                  (0x00000000U)

/* TX3_BIST_LNA_CTRL */

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MASK              (0x0000001FU)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MASK             (0x000003E0U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MAX              (0x0000001FU)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MASK      (0x00000400U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_SHIFT     (0x0000000AU)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MASK        (0x0001F800U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_SHIFT       (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MAX         (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MASK        (0x00020000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_SHIFT       (0x00000011U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MASK     (0x00040000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_SHIFT    (0x00000012U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MASK       (0x00380000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MAX        (0x00000007U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MASK     (0x00C00000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_SHIFT    (0x00000016U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MAX      (0x00000003U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_RESERVED0_MASK                       (0xFE000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_RESERVED0_SHIFT                      (0x00000019U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_RESERVED0_MAX                        (0x0000007FU)

#define CSL_AR_RFANACIO_TX3_BIST_LNA_CTRL_RESETVAL                             (0x00000000U)

/* TX3_PS_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MASK (0x0000003FU)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MAX (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MASK (0x00000FC0U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_SHIFT (0x00000006U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MASK (0x0003F000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_RESERVED0_MASK                    (0xFFFC0000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_RESERVED0_SHIFT                   (0x00000012U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_RESERVED0_MAX                     (0x00003FFFU)

#define CSL_AR_RFANACIO_TX3_PS_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX3_PS_TMUXCTRL */

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MASK              (0x0001U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_SHIFT             (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MASK              (0x0002U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_SHIFT             (0x0001U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MASK              (0x0004U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_SHIFT             (0x0002U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MASK              (0x0008U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_SHIFT             (0x0003U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MASK                 (0x0010U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_SHIFT                (0x0004U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MASK             (0x0020U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_SHIFT            (0x0005U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MASK             (0x0040U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_SHIFT            (0x0006U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MASK             (0x0080U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_SHIFT            (0x0007U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MASK             (0x0100U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_SHIFT            (0x0008U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MASK               (0x0200U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_SHIFT              (0x0009U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MASK               (0x0400U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_SHIFT              (0x000AU)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MASK            (0x0800U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_SHIFT           (0x000BU)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MASK         (0x1000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_SHIFT        (0x000CU)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_RESETVAL     (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MAX          (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MASK                 (0x2000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_SHIFT                (0x000DU)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX3_PS_TMUXCTRL_RESETVAL                               (0x0000U)

/* TX3_CTRL_RC_FILT */

#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MASK           (0x03U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_SHIFT          (0x00U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_RESETVAL       (0x00U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MAX            (0x03U)

#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_RESERVED0_MASK                        (0x0CU)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_RESERVED0_SHIFT                       (0x02U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_RESERVED0_RESETVAL                    (0x00U)
#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_RESERVED0_MAX                         (0x03U)

#define CSL_AR_RFANACIO_TX3_CTRL_RC_FILT_RESETVAL                              (0x00U)

/* TX3_IQGENLDO_CTRL */

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MASK         (0x00000001U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_SHIFT        (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MASK           (0x000000FEU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MAX            (0x0000007FU)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MASK     (0x00000100U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_SHIFT    (0x00000008U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MASK      (0x00000200U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_SHIFT     (0x00000009U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MASK     (0x00000800U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_SHIFT    (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MASK (0x00002000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED0_MASK                       (0x0000C000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED0_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED0_MAX                        (0x00000003U)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED1_MASK                       (0xFFFF0000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED1_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESERVED1_MAX                        (0x0000FFFFU)

#define CSL_AR_RFANACIO_TX3_IQGENLDO_CTRL_RESETVAL                             (0x00000000U)

/* TX3_IQGEN_BIAS_CTRL */

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MASK              (0x0000003FU)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MASK              (0x00000FC0U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX34_ROOT_AMP_MASK          (0x0003F000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX34_ROOT_AMP_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX34_ROOT_AMP_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX34_ROOT_AMP_MAX           (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX3_AMP_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX3_AMP_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX3_AMP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_LODIST_TX3_AMP_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED1_MASK                     (0x3F000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED1_SHIFT                    (0x00000018U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED1_MAX                      (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED2_MASK                     (0x40000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED2_SHIFT                    (0x0000001EU)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED2_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED3_MASK                     (0x80000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED3_SHIFT                    (0x0000001FU)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED3_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESERVED3_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_IQGEN_BIAS_CTRL_RESETVAL                           (0x00000000U)

/* TX3_DAC_ANALDO_CTRL */

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MASK           (0x0000001EU)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MASK (0x00010000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MASK (0x00020000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MASK      (0x00040000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_SHIFT     (0x00000012U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MASK        (0x00100000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_SHIFT       (0x00000014U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_ANALDO_CTRL_RESETVAL                           (0x00000000U)

/* TX3_DAC_DIGLDO_CTRL */

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MASK           (0x0000003EU)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MAX            (0x0000001FU)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MASK        (0x00000100U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MASK (0x00000200U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_SHIFT (0x00000009U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MASK       (0x00000800U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_SHIFT      (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MASK   (0x001FE000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_SHIFT  (0x0000000DU)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MAX    (0x000000FFU)

#define CSL_AR_RFANACIO_TX3_DAC_DIGLDO_CTRL_RESETVAL                           (0x00000000U)

/* TX3_PA_TESTMUX_CTRL */

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED1_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED1_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MASK         (0x00000002U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_SHIFT        (0x00000001U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MASK       (0x00000004U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_SHIFT      (0x00000002U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MASK       (0x00000008U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_SHIFT      (0x00000003U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MASK       (0x00000010U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MASK       (0x00000020U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MASK      (0x00000040U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_SHIFT     (0x00000006U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MASK         (0x00000100U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MASK       (0x00000200U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_SHIFT      (0x00000009U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MASK       (0x00000400U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_SHIFT      (0x0000000AU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MASK      (0x00001000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_SHIFT     (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MASK         (0x00002000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_SHIFT        (0x0000000DU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MASK       (0x00004000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_SHIFT      (0x0000000EU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MASK      (0x00010000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_SHIFT     (0x00000010U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MASK        (0x00040000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_SHIFT       (0x00000012U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MASK       (0x00100000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED0_MASK                     (0x01E00000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED0_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESERVED0_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_TX3_PA_TESTMUX_CTRL_RESETVAL                           (0x00000000U)

/* TX3_PA_SPARES */

#define CSL_AR_RFANACIO_TX3_PA_SPARES_RESERVED0_MASK                           (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_TX3_PA_SPARES_RESERVED0_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_SPARES_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_SPARES_RESERVED0_MAX                            (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_TX3_PA_SPARES_RESETVAL                                 (0x00000000U)

/* TX3_PA_EN */

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG1_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG1_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG1_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG2_MASK                          (0x00000002U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG2_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG2_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3L_MASK                         (0x00000004U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3L_SHIFT                        (0x00000002U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3R_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3R_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG3R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4L_MASK                         (0x00000010U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4L_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4R_MASK                         (0x00000020U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4R_SHIFT                        (0x00000005U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_STG4R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_BIAS_MASTER_EN_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_BIAS_MASTER_EN_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_BIAS_MASTER_EN_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_BIAS_MASTER_EN_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_CTRL_PATH_MASK                     (0x00000080U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_CTRL_PATH_SHIFT                    (0x00000007U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_CTRL_PATH_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_PA_ENABLE_CTRL_PATH_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED0_MASK                               (0x00000F00U)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED0_SHIFT                              (0x00000008U)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED0_MAX                                (0x0000000FU)

#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED1_MASK                               (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED1_SHIFT                              (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_EN_RESERVED1_MAX                                (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX3_PA_EN_RESETVAL                                     (0x00000000U)

/* TX3_PA_BIASCTRL_REG2 */

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MASK                (0x0000003FU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4L_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MASK                (0x00000FC0U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4R_SHIFT               (0x00000006U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_RESERVED0_MASK                    (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_RESERVED0_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_RESERVED0_MAX                     (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG2_RESETVAL                          (0x00000000U)

/* TX3_PA_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG1_MASK                 (0x0000003FU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG1_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG1_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG2_MASK                 (0x00000FC0U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG2_SHIFT                (0x00000006U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG2_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MASK                (0x0003F000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3L_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3R_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_RESERVED0_MASK                    (0x1F000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_RESERVED0_SHIFT                   (0x00000018U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_RESERVED0_MAX                     (0x0000001FU)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MASK                (0xE0000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_IBIAS_TEST_SHIFT               (0x0000001DU)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_IBIAS_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_TX3_PA_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX3_DAC_CONFIG */

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_EN_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_EN_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_EN_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_EN_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_CTRL_MASK                          (0x0000000EU)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_CTRL_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_CTRL_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_BIAS_CTRL_MAX                           (0x00000007U)

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_ICNTRL_MASK                         (0x00000FF0U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_ICNTRL_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_ICNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_ICNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_QCNTRL_MASK                         (0x000FF000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_QCNTRL_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_QCNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_DAC_QCNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_RESERVED_MASK                           (0xFFF00000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_RESERVED_SHIFT                          (0x00000014U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_RESERVED_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_RESERVED_MAX                            (0x00000FFFU)

#define CSL_AR_RFANACIO_TX3_DAC_CONFIG_RESETVAL                                (0x00000000U)

/* TX3_DAC_CTRL */

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKDLY_MASK                           (0x00000007U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKDLY_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKINVEN_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKINVEN_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ILATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLKINV_MASK                              (0x00000010U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLKINV_SHIFT                             (0x00000004U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_ICAL_MASK                              (0x00000020U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_ICAL_SHIFT                             (0x00000005U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_ICAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_ICAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICALCLKSEL_MASK                           (0x000003C0U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICALCLKSEL_SHIFT                          (0x00000006U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_IDEC_MASK                              (0x00000400U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_IDEC_SHIFT                             (0x0000000AU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_IDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_IDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACI_RESETB_MASK                          (0x00000800U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACI_RESETB_SHIFT                         (0x0000000BU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACI_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACI_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVI_DEC_MASK                          (0x00001000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVI_DEC_SHIFT                         (0x0000000CU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVI_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVI_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLK_DEC_EN_MASK                          (0x00002000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLK_DEC_EN_SHIFT                         (0x0000000DU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ICLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ISIGN_EN_MASK                             (0x00004000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ISIGN_EN_SHIFT                            (0x0000000EU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ISIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_ISIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKDLY_MASK                           (0x00038000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKDLY_SHIFT                          (0x0000000FU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKINVEN_MASK                         (0x00040000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKINVEN_SHIFT                        (0x00000012U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QLATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLKINV_MASK                              (0x00080000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLKINV_SHIFT                             (0x00000013U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QCAL_MASK                              (0x00100000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QCAL_SHIFT                             (0x00000014U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QCAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QCAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCALCLKSEL_MASK                           (0x01E00000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCALCLKSEL_SHIFT                          (0x00000015U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QDEC_MASK                              (0x02000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QDEC_SHIFT                             (0x00000019U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_EN_QDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACQ_RESETB_MASK                          (0x04000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACQ_RESETB_SHIFT                         (0x0000001AU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACQ_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_DACQ_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVQ_DEC_MASK                          (0x08000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVQ_DEC_SHIFT                         (0x0000001BU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVQ_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_CLKDIVQ_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLK_DEC_EN_MASK                          (0x10000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLK_DEC_EN_SHIFT                         (0x0000001CU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QCLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QSIGN_EN_MASK                             (0x20000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QSIGN_EN_SHIFT                            (0x0000001DU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QSIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_QSIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_RESERVED_MASK                             (0xC0000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_RESERVED_SHIFT                            (0x0000001EU)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX3_DAC_CTRL_RESERVED_MAX                              (0x00000003U)

#define CSL_AR_RFANACIO_TX3_DAC_CTRL_RESETVAL                                  (0x00000000U)

/* LODIST_CTRL5 */

#define CSL_AR_RFANACIO_LODIST_CTRL5_RESERVED0_MASK                            (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_LODIST_CTRL5_RESERVED0_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL5_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL5_RESERVED0_MAX                             (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_LODIST_CTRL5_RESETVAL                                  (0x00000000U)

/* LODIST_ATEST_CTRL5 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_0_MASK           (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_0_SHIFT          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_0_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_1_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_1_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_2_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_2_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_3_MASK           (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_3_SHIFT          (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_3_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_3_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_4_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_4_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_4_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_4_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_5_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_5_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_5_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_5_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_6_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_6_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_6_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_6_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_7_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_7_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_7_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_7_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_8_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_8_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_8_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_8_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_9_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_9_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_9_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_9_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_10_MASK          (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_10_SHIFT         (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_10_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_10_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_11_MASK          (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_11_SHIFT         (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_11_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_11_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_12_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_12_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_12_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_12_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_13_MASK          (0x00002000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_13_SHIFT         (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_13_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_13_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_14_MASK          (0x00004000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_14_SHIFT         (0x0000000EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_14_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_14_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_15_MASK          (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_15_SHIFT         (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_15_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_15_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_16_MASK          (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_16_SHIFT         (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_16_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_16_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_17_MASK          (0x00020000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_17_SHIFT         (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_17_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_17_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_18_MASK          (0x00040000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_18_SHIFT         (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_18_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_18_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_19_MASK          (0x00080000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_19_SHIFT         (0x00000013U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_19_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_19_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_20_MASK          (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_20_SHIFT         (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_20_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_20_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_21_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_21_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_21_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_21_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED0_MASK                      (0x00C00000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED0_SHIFT                     (0x00000016U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED0_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_24_MASK          (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_24_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_24_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_24_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED1_MASK                      (0x3E000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED1_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESERVED1_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_30_MASK          (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_30_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_30_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_30_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_31_MASK          (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_31_SHIFT         (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_31_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_TX12_CLUSTER_ATEST_31_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL5_RESETVAL                            (0x00000000U)

/* LODIST_CTRL6 */

#define CSL_AR_RFANACIO_LODIST_CTRL6_RESERVED0_MASK                            (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_LODIST_CTRL6_RESERVED0_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL6_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL6_RESERVED0_MAX                             (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_LODIST_CTRL6_RESETVAL                                  (0x00000000U)

/* LODIST_ATEST_CTRL6 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_0_MASK           (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_0_SHIFT          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_0_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_1_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_1_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_2_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_2_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_3_MASK           (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_3_SHIFT          (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_3_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_3_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_4_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_4_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_4_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_4_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_5_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_5_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_5_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_5_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_6_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_6_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_6_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_6_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_7_MASK           (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_7_SHIFT          (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_7_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_7_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_8_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_8_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_8_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_8_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_9_MASK           (0x00000200U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_9_SHIFT          (0x00000009U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_9_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_9_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_10_MASK          (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_10_SHIFT         (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_10_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_10_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_11_MASK          (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_11_SHIFT         (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_11_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_11_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_12_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_12_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_12_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_12_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_13_MASK          (0x00002000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_13_SHIFT         (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_13_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_13_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_14_MASK          (0x00004000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_14_SHIFT         (0x0000000EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_14_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_14_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_15_MASK          (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_15_SHIFT         (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_15_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_15_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_16_MASK          (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_16_SHIFT         (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_16_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_16_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_17_MASK          (0x00020000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_17_SHIFT         (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_17_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_17_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_18_MASK          (0x00040000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_18_SHIFT         (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_18_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_18_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_19_MASK          (0x00080000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_19_SHIFT         (0x00000013U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_19_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_19_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_20_MASK          (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_20_SHIFT         (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_20_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_20_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_21_MASK          (0x00200000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_21_SHIFT         (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_21_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_21_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED0_MASK                      (0x00C00000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED0_SHIFT                     (0x00000016U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED0_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_24_MASK          (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_24_SHIFT         (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_24_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_24_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED1_MASK                      (0x3E000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED1_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESERVED1_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_30_MASK          (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_30_SHIFT         (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_30_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_30_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_31_MASK          (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_31_SHIFT         (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_31_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_TX34_CLUSTER_ATEST_31_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL6_RESETVAL                            (0x00000000U)

/* LODIST_CTRL2 */

#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_EN_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_EN_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_CTRL_MASK                   (0x000000FEU)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_CTRL_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_CTRL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG4_BIAS_CTRL_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED0_MASK                            (0x00000300U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED0_SHIFT                           (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED0_MAX                             (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_EN_MASK                     (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_EN_SHIFT                    (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_EN_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_EN_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_CTRL_MASK                   (0x0003F800U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_CTRL_SHIFT                  (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_CTRL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_COM_STG5_BIAS_CTRL_MAX                    (0x0000007FU)

#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED1_MASK                            (0x000C0000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED1_SHIFT                           (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED1_MAX                             (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED2_MASK                            (0x3FF00000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED2_SHIFT                           (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED2_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED2_MAX                             (0x000003FFU)

#define CSL_AR_RFANACIO_LODIST_CTRL2_MULT_COM_PWR_SWITCH_EN_MASK               (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_MULT_COM_PWR_SWITCH_EN_SHIFT              (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_CTRL2_MULT_COM_PWR_SWITCH_EN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_MULT_COM_PWR_SWITCH_EN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED3_MASK                            (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED3_SHIFT                           (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED3_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL2_RESERVED3_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL2_RESETVAL                                  (0x00000000U)

/* LODIST_ATEST_CTRL2 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2_RESERVED0_MASK                      (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2_RESERVED0_SHIFT                     (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2_RESERVED0_MAX                       (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL2_RESETVAL                            (0x00000000U)

/* REFSYS_CTRL_REG */

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE0_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE0_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE0_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE0_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE1_MASK                   (0x00000002U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE1_SHIFT                  (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE1_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE1_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE2_MASK                   (0x00000004U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE2_SHIFT                  (0x00000002U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE2_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE2_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE3_MASK                   (0x00000008U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE3_SHIFT                  (0x00000003U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE3_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE3_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE4_MASK                   (0x000001F0U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE4_SHIFT                  (0x00000004U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE4_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE4_MAX                    (0x0000001FU)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE5_MASK                   (0x00003E00U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE5_SHIFT                  (0x00000009U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE5_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE5_MAX                    (0x0000001FU)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE6_MASK                   (0x0007C000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE6_SHIFT                  (0x0000000EU)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE6_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE6_MAX                    (0x0000001FU)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE7_MASK                   (0x00080000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE7_SHIFT                  (0x00000013U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE7_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE7_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE8_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE8_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE8_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE8_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE9_MASK                   (0x00200000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE9_SHIFT                  (0x00000015U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE9_RESETVAL               (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE9_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_LODIST_IBIAS_EN_MASK                   (0x00400000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_LODIST_IBIAS_EN_SHIFT                  (0x00000016U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_LODIST_IBIAS_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_LODIST_IBIAS_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_TX_TOP_IBIAS_EN_MASK                   (0x00800000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_TX_TOP_IBIAS_EN_SHIFT                  (0x00000017U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_TX_TOP_IBIAS_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_TX_TOP_IBIAS_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE10_MASK                  (0x01000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE10_SHIFT                 (0x00000018U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE10_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE10_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_IDIODE_EN_MASK                         (0x02000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_IDIODE_EN_SHIFT                        (0x00000019U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_IDIODE_EN_RESETVAL                     (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_IDIODE_EN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RX_TOP_IBIAS_EN_MASK                   (0x04000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RX_TOP_IBIAS_EN_SHIFT                  (0x0000001AU)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RX_TOP_IBIAS_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RX_TOP_IBIAS_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE11_MASK                  (0x78000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE11_SHIFT                 (0x0000001BU)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE11_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_BSS_DO_NOT_USE11_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RESERVED0_MASK                         (0x80000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RESERVED0_SHIFT                        (0x0000001FU)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RESERVED0_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_CTRL_REG_RESETVAL                               (0x02200000U)

/* TW_CTRL_REG */

#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_EN_MASK                                (0x00000001U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_EN_SHIFT                               (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_EN_RESETVAL                            (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_EN_MAX                                 (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED_MASK                              (0x00000002U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED_SHIFT                             (0x00000001U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_RESET_MASK                             (0x00000004U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_RESET_SHIFT                            (0x00000002U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_RESET_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_RESET_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_INP_BUF_EN_MASK                        (0x00000008U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_INP_BUF_EN_SHIFT                       (0x00000003U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_INP_BUF_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_INP_BUF_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_BUF_EN_MASK                        (0x00000010U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_BUF_EN_SHIFT                       (0x00000004U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_BUF_EN_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_BUF_EN_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_SEL_2_0_MASK                       (0x000000E0U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_SEL_2_0_SHIFT                      (0x00000005U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_SEL_2_0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ADC_REF_SEL_2_0_MAX                        (0x00000007U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_DIFF_INP_BUF_EN_MASK                    (0x00000100U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_DIFF_INP_BUF_EN_SHIFT                   (0x00000008U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_DIFF_INP_BUF_EN_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_DIFF_INP_BUF_EN_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_SE_INP_BUF_EN_MASK                      (0x00000200U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_SE_INP_BUF_EN_SHIFT                     (0x00000009U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_SE_INP_BUF_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_TS_SE_INP_BUF_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_IFORCE_EXT_CTRL_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_IFORCE_EXT_CTRL_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_IFORCE_EXT_CTRL_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_IFORCE_EXT_CTRL_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_VREF_EXT_CTRL_MASK                         (0x00000800U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VREF_EXT_CTRL_SHIFT                        (0x0000000BU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VREF_EXT_CTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VREF_EXT_CTRL_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_VIN_EXT_CTRL_MASK                          (0x00001000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VIN_EXT_CTRL_SHIFT                         (0x0000000CU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VIN_EXT_CTRL_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_VIN_EXT_CTRL_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_BYPASS_MASK                   (0x00002000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_BYPASS_SHIFT                  (0x0000000DU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_BYPASS_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_BYPASS_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_EN_MASK                       (0x00004000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_EN_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_EN_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_ANA_TMUX_BUF_EN_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TW_CTRL_REG_RTRIM_TW_4_0_MASK                          (0x000F8000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RTRIM_TW_4_0_SHIFT                         (0x0000000FU)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RTRIM_TW_4_0_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RTRIM_TW_4_0_MAX                           (0x0000001FU)

#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED0_MASK                             (0xFFF00000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED0_SHIFT                            (0x00000014U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED0_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TW_CTRL_REG_RESERVED0_MAX                              (0x00000FFFU)

#define CSL_AR_RFANACIO_TW_CTRL_REG_RESETVAL                                   (0x00000000U)

/* TW_ANA_TMUX_CTRL */

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_WU_ANA_TEST_OUT_1P8V_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_WU_ANA_TEST_OUT_1P8V_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_WU_ANA_TEST_OUT_1P8V_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_WU_ANA_TEST_OUT_1P8V_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_REFSYS_TEST_OUT_1P8V_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_REFSYS_TEST_OUT_1P8V_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_REFSYS_TEST_OUT_1P8V_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_REFSYS_TEST_OUT_1P8V_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_2_MASK                     (0x00000004U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_2_SHIFT                    (0x00000002U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_2_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_3_MASK                     (0x00000008U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_3_SHIFT                    (0x00000003U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_3_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_3_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_4_MASK                     (0x00000010U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_4_SHIFT                    (0x00000004U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_4_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_4_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_5_MASK                     (0x00000020U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_5_SHIFT                    (0x00000005U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_5_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_PM_ANA_INP_5_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_S_BUFF_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_S_BUFF_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_S_BUFF_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_S_BUFF_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_W_BUFF_MASK                       (0x00000080U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_W_BUFF_SHIFT                      (0x00000007U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_W_BUFF_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_W_BUFF_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DC_BIST_BUF_INP_1P8V_MASK             (0x00000100U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DC_BIST_BUF_INP_1P8V_SHIFT            (0x00000008U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DC_BIST_BUF_INP_1P8V_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DC_BIST_BUF_INP_1P8V_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_BUF_OUT_1P8V_MASK                 (0x00000200U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_BUF_OUT_1P8V_SHIFT                (0x00000009U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_BUF_OUT_1P8V_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_BUF_OUT_1P8V_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_REF_BUF_OUT_MASK                  (0x00000400U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_REF_BUF_OUT_SHIFT                 (0x0000000AU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_REF_BUF_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ADC_REF_BUF_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DELVBE_BUFF_OUT_MASK                  (0x00000800U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DELVBE_BUFF_OUT_SHIFT                 (0x0000000BU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DELVBE_BUFF_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_DELVBE_BUFF_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_STRONG_MASK                    (0x00001000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_STRONG_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_STRONG_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_STRONG_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_WEAK_MASK                      (0x00002000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_WEAK_SHIFT                     (0x0000000DU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_WEAK_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_VBE_TS_WEAK_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_MASK                              (0x00004000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_SHIFT                             (0x0000000EU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BIST_MUX_OUT_1P8V_MASK                (0x00008000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BIST_MUX_OUT_1P8V_SHIFT               (0x0000000FU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BIST_MUX_OUT_1P8V_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BIST_MUX_OUT_1P8V_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_I2V_SENSE_MASK                        (0x00010000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_I2V_SENSE_SHIFT                       (0x00000010U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_I2V_SENSE_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_I2V_SENSE_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_TMUX_BUF_OUT_EN_MASK                  (0x00020000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_TMUX_BUF_OUT_EN_SHIFT                 (0x00000011U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_TMUX_BUF_OUT_EN_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_TMUX_BUF_OUT_EN_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ATESTV_LDO_MASK                       (0x00040000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ATESTV_LDO_SHIFT                      (0x00000012U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ATESTV_LDO_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ATESTV_LDO_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE0_MASK                  (0x00080000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE0_SHIFT                 (0x00000013U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE0_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE1_MASK                  (0x00100000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE1_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE1_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE2_MASK                  (0x00200000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE2_SHIFT                 (0x00000015U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE2_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_BSS_DO_NOT_USE2_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_UNBUF_MASK                        (0x00400000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_UNBUF_SHIFT                       (0x00000016U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_UNBUF_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ODP_UNBUF_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_RESERVED0_MASK                        (0x3F800000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_RESERVED0_SHIFT                       (0x00000017U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_RESERVED0_MAX                         (0x0000007FU)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_CLK_TMUX_ESD_CTRL_MASK                (0x40000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_CLK_TMUX_ESD_CTRL_SHIFT               (0x0000001EU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_CLK_TMUX_ESD_CTRL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_CLK_TMUX_ESD_CTRL_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ANA_TEST_EN_MASK                      (0x80000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ANA_TEST_EN_SHIFT                     (0x0000001FU)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ANA_TEST_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_ANA_TEST_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TW_ANA_TMUX_CTRL_RESETVAL                              (0x00000000U)

/* RFANA_TOP_CTRL1 */

#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1_RESERVED0_MASK                         (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1_RESERVED0_SHIFT                        (0x00000000U)
#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1_RESERVED0_MAX                          (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RFANA_TOP_CTRL1_RESETVAL                               (0x00000000U)

/* LODIST_LDO_CTRL1 */

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_MASK                (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TRIM_MASK              (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TRIM_SHIFT             (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TRIM_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TRIM_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_ENZ_LOW_BW_CAP_MASK    (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_ENZ_LOW_BW_CAP_SHIFT   (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_ENZ_LOW_BW_CAP_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_ENZ_LOW_BW_CAP_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_SC_MASK             (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_SC_SHIFT            (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_SC_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_EN_SC_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BYPASS_MASK            (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BYPASS_SHIFT           (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BYPASS_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BYPASS_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BW_CTRL_MASK           (0x00000700U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BW_CTRL_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BW_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_BW_CTRL_MAX            (0x00000007U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_SC_BIAS_MASK           (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_SC_BIAS_SHIFT          (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_SC_BIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_SC_BIAS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_PMOS_PD_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_PMOS_PD_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_PMOS_PD_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_PMOS_PD_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TLOAD_CTRL_MASK        (0x0000E000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TLOAD_CTRL_SHIFT       (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TLOAD_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_LDO_TLOAD_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_EN_MASK             (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_EN_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_CFG_MASK            (0x00FE0000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_CFG_SHIFT           (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_CFG_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_DAC_CFG_MAX             (0x0000007FU)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_ANATEST1_MASK      (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_ANATEST1_SHIFT     (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_ANATEST1_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_ANATEST1_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VIN_ANATEST34_MASK      (0x02000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VIN_ANATEST34_SHIFT     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VIN_ANATEST34_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VIN_ANATEST34_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_STG1_ANATEST34_MASK (0x04000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_STG1_ANATEST34_SHIFT (0x0000001AU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_STG1_ANATEST34_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VOUT_STG1_ANATEST34_MAX (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VREF_ANATEST1_MASK      (0x08000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VREF_ANATEST1_SHIFT     (0x0000001BU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VREF_ANATEST1_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_VREF_ANATEST1_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_ITEST_ANATEST1_MASK     (0x10000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_ITEST_ANATEST1_SHIFT    (0x0000001CU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_ITEST_ANATEST1_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_ITEST_ANATEST1_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_INC_BIAS_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_INC_BIAS_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_INC_BIAS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_PD_INC_BIAS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_RESERVED0_MASK                        (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_RESERVED0_SHIFT                       (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_RESERVED0_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_RESERVED0_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_FASTEN_MASK_MASK           (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_FASTEN_MASK_SHIFT          (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_FASTEN_MASK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_MULT_CHAIN_FASTEN_MASK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_LDO_CTRL1_RESETVAL                              (0x00000000U)

/* LODIST_CTRL1 */

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_EN_MASK                   (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_EN_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CAS_EN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CAS_EN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CAS_EN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CAS_EN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CAS_EN_MASK          (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CAS_EN_SHIFT         (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CAS_EN_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CAS_EN_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CASBIAS_CTRL_MASK         (0x00000078U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CASBIAS_CTRL_SHIFT        (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CASBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_CASBIAS_CTRL_MAX          (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CASBIAS_CTRL_MASK    (0x00000380U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CASBIAS_CTRL_SHIFT   (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CASBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_CASBIAS_CTRL_MAX     (0x00000007U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_EN_MASK                      (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_EN_SHIFT                     (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_EN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_EN_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_BIAS_MASK                    (0x0001F800U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_BIAS_SHIFT                   (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_BIAS_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_STG2_40G_AMP_BIAS_MAX                     (0x0000003FU)

#define CSL_AR_RFANACIO_LODIST_CTRL1_RESERVED0_MASK                            (0x000E0000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_RESERVED0_SHIFT                           (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_RESERVED0_MAX                             (0x00000007U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_EN_MASK                   (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_EN_SHIFT                  (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_BIAS_CTRL_MASK            (0x07E00000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_BIAS_CTRL_SHIFT           (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_BIAS_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_BIAS_CTRL_MAX             (0x0000003FU)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_CASBIAS_CTRL_MASK         (0x38000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_CASBIAS_CTRL_SHIFT        (0x0000001BU)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_CASBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_40G_TO_80G_CASBIAS_CTRL_MAX          (0x00000007U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_CHAIN_BIAS_EN_MASK                   (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_CHAIN_BIAS_EN_SHIFT                  (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_CHAIN_BIAS_EN_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_CHAIN_BIAS_EN_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_EN_MASK              (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_EN_SHIFT             (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_CTRL1_MULT_20G_TO_40G_SYNC_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_CTRL1_RESETVAL                                  (0x00000000U)

/* LODIST_ATEST_CTRL1 */

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_0_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_0_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_0_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_0_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_1_MASK             (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_1_SHIFT            (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_1_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_1_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_2_MASK             (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_2_SHIFT            (0x00000002U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_2_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_2_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_3_MASK             (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_3_SHIFT            (0x00000003U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_3_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_3_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_4_MASK             (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_4_SHIFT            (0x00000004U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_4_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_4_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_5_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_5_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_5_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_5_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_6_MASK             (0x00000040U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_6_SHIFT            (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_6_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_6_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_7_MASK             (0x00000080U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_7_SHIFT            (0x00000007U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_7_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_7_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_8_MASK             (0x00000100U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_8_SHIFT            (0x00000008U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_8_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_8_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_9_MASK             (0x00000200U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_9_SHIFT            (0x00000009U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_9_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_9_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_10_MASK            (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_10_SHIFT           (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_10_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_10_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_11_MASK            (0x00000800U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_11_SHIFT           (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_11_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_11_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_12_MASK            (0x00001000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_12_SHIFT           (0x0000000CU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_12_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_12_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_13_MASK            (0x00002000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_13_SHIFT           (0x0000000DU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_13_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_13_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_14_MASK            (0x00004000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_14_SHIFT           (0x0000000EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_14_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_14_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_15_MASK            (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_15_SHIFT           (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_15_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_15_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_16_MASK            (0x00010000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_16_SHIFT           (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_16_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_16_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_17_MASK            (0x00020000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_17_SHIFT           (0x00000011U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_17_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_17_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_18_MASK            (0x00040000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_18_SHIFT           (0x00000012U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_18_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_18_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_19_MASK            (0x00080000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_19_SHIFT           (0x00000013U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_19_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_19_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_20_MASK            (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_20_SHIFT           (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_20_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_20_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_21_MASK            (0x00200000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_21_SHIFT           (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_21_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_21_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED0_MASK                      (0x00C00000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED0_SHIFT                     (0x00000016U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED0_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_24_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_24_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_24_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_24_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED1_MASK                      (0x3E000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED1_SHIFT                     (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESERVED1_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_30_MASK            (0x40000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_30_SHIFT           (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_30_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_30_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_31_MASK            (0x80000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_31_SHIFT           (0x0000001FU)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_31_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_MULT_CHAIN_ATEST_31_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_ATEST_CTRL1_RESETVAL                            (0x00000000U)

/* LODIST_GEN_SPARES */

#define CSL_AR_RFANACIO_LODIST_GEN_SPARES_RESERVED0_MASK                       (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_LODIST_GEN_SPARES_RESERVED0_SHIFT                      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_GEN_SPARES_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_GEN_SPARES_RESERVED0_MAX                        (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_LODIST_GEN_SPARES_RESETVAL                             (0x00000000U)

/* CLK_CTRL_REG1_SYNC_20G */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_4_0_MASK    (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_4_0_SHIFT   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_4_0_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_4_0_MAX     (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_7_5_MASK    (0x000000E0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_7_5_SHIFT   (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_7_5_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_7_5_MAX     (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_12_8_MASK   (0x00001F00U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_12_8_SHIFT  (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_12_8_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_12_8_MAX    (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_15_13_MASK  (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_15_13_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_15_13_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_15_13_MAX   (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_20_16_MASK  (0x001F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_20_16_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_20_16_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_20_16_MAX   (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_23_21_MASK  (0x00E00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_23_21_SHIFT (0x00000015U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_23_21_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_23_21_MAX   (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_28_24_MASK  (0x1F000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_28_24_SHIFT (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_28_24_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_28_24_MAX   (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_31_29_MASK  (0xE0000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_31_29_SHIFT (0x0000001DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_31_29_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_SYNC_20G_BIAS_CTRL1_31_29_MAX   (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNC_20G_RESETVAL                        (0x00000000U)

/* CLK_CTRL_REG2_SYNC_20G */

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_4_0_MASK    (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_4_0_SHIFT   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_4_0_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_4_0_MAX     (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_7_5_MASK    (0x000000E0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_7_5_SHIFT   (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_7_5_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_7_5_MAX     (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_12_8_MASK   (0x00001F00U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_12_8_SHIFT  (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_12_8_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_12_8_MAX    (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_15_13_MASK  (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_15_13_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_15_13_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_15_13_MAX   (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_20_16_MASK  (0x001F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_20_16_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_20_16_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_SYNC_20G_BIAS_CTRL2_20_16_MAX   (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_RESERVED1_MASK                  (0x7FE00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_RESERVED1_SHIFT                 (0x00000015U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_RESERVED1_MAX                   (0x000003FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_CLK_BIST_DISABLE_SYNC_20G_MASK  (0x80000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_CLK_BIST_DISABLE_SYNC_20G_SHIFT (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_CLK_BIST_DISABLE_SYNC_20G_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_CLK_BIST_DISABLE_SYNC_20G_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNC_20G_RESETVAL                        (0x00000000U)

/* CLK_CTRL_REG3_SYNC_20G */

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED0_MASK                  (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED0_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED0_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED0_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_TEST_MUX_CTRL_MASK              (0x000000F0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_TEST_MUX_CTRL_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_TEST_MUX_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_TEST_MUX_CTRL_MAX               (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED1_MASK                  (0x00000300U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED1_SHIFT                 (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED1_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED1_MAX                   (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_10_MASK      (0x00000400U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_10_SHIFT     (0x0000000AU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_10_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_10_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_11_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_11_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_11_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_11_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_15_12_MASK   (0x0000F000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_15_12_SHIFT  (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_15_12_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTRL_REG3_SYNC_20G_15_12_MAX    (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_0_MASK         (0x00010000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_0_SHIFT        (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_0_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_0_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_7_TO_1_MASK    (0x00FE0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_7_TO_1_SHIFT   (0x00000011U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_7_TO_1_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_7_TO_1_MAX     (0x0000007FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_8_MASK         (0x01000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_8_SHIFT        (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_8_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_8_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_9_MASK         (0x02000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_9_SHIFT        (0x00000019U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_9_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_9_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_10_MASK        (0x04000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_10_SHIFT       (0x0000001AU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_10_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_CTL_PDLNA_STATIC_10_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED2_MASK                  (0xF8000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED2_SHIFT                 (0x0000001BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED2_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESERVED2_MAX                   (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNC_20G_RESETVAL                        (0x00000000U)

/* CLK_CTRL_REG2_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_BIAS_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_BIAS_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_BIAS_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_BIAS_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_VCOBUF_MASK           (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_VCOBUF_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_VCOBUF_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_VCOBUF_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF1_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF1_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF2_MASK           (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF2_SHIFT          (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_IPBUF2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF1_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF1_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF1_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF1_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF1_MASK        (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF1_SHIFT       (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF1_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF1_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF2_MASK           (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF2_SHIFT          (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_OPBUF2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF2_MASK        (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF2_SHIFT       (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF2_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_ALTOPBUF2_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_MIX_MASK              (0x00000100U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_MIX_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_MIX_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SYNC_20G_MIX_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_INPUT_DIVIDER_MASK      (0x00000200U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_INPUT_DIVIDER_SHIFT     (0x00000009U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_INPUT_DIVIDER_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_INPUT_DIVIDER_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_CFG2_HR_DIVIDER_MASK    (0x00000400U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_CFG2_HR_DIVIDER_SHIFT   (0x0000000AU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_CFG2_HR_DIVIDER_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_CFG2_HR_DIVIDER_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SOCC_S2P_RESETZ_BYPASS_MASK    (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SOCC_S2P_RESETZ_BYPASS_SHIFT   (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SOCC_S2P_RESETZ_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_EN_SOCC_S2P_RESETZ_BYPASS_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_S2P_RESETZ_BYPASS_MASK  (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_S2P_RESETZ_BYPASS_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_S2P_RESETZ_BYPASS_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_CTRL_SOCC_S2P_RESETZ_BYPASS_MAX   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED0_MASK                    (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED0_SHIFT                   (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED0_MAX                     (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_TEST_CTRL_MASK                    (0x00FF0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_TEST_CTRL_SHIFT                   (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_TEST_CTRL_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_TEST_CTRL_MAX                     (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED1_MASK                    (0x07000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED1_SHIFT                   (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED1_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED1_MAX                     (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_REF_BYP_CTRL_MASK                 (0x08000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_REF_BYP_CTRL_SHIFT                (0x0000001BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_REF_BYP_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_REF_BYP_CTRL_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED2_MASK                    (0x10000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED2_SHIFT                   (0x0000001CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED2_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED2_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_ENB_BIAS_MASK                     (0x20000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_ENB_BIAS_SHIFT                    (0x0000001DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_ENB_BIAS_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_ENB_BIAS_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED3_MASK                    (0xC0000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED3_SHIFT                   (0x0000001EU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED3_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESERVED3_MAX                     (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_CLKTOP_RESETVAL                          (0x00000000U)

/* CLK_CTRL_REG8_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENZ_LOW_BW_CAP_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENZ_LOW_BW_CAP_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENZ_LOW_BW_CAP_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENZ_LOW_BW_CAP_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MASK         (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_SCPRT_IBIAS_CTRL_SHIFT        (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_SCPRT_IBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MASK     (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_SHIFT    (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TLOAD_CTRL_MASK               (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TLOAD_CTRL_SHIFT              (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TLOAD_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TLOAD_CTRL_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TESTMUX_CTRL_MASK             (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TESTMUX_CTRL_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_TESTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_BISTMUX_CTRL_MASK             (0x00F00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_BISTMUX_CTRL_SHIFT            (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_BISTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_BISTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_RESERVED0_MASK                (0xFF000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_RESERVED0_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_RESERVED0_MAX                 (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG8_LDO_CLKTOP_RESETVAL                      (0x00000000U)

/* CLK_CTRL_REG6_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED1_MASK                (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED1_SHIFT               (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED1_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED1_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED2_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED2_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED2_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED3_MASK                (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED3_SHIFT               (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED3_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESERVED3_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_TESTMUX_CTRL_MASK             (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_TESTMUX_CTRL_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_TESTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_BIST_MUX_CONTROL_MASK         (0xFFF00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_BIST_MUX_CONTROL_SHIFT        (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_BIST_MUX_CONTROL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_BIST_MUX_CONTROL_MAX          (0x00000FFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_LDO_CLKTOP_RESETVAL                      (0x00000000U)

/* CLK_CTRL_REG6_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_SEL_MASK            (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_SEL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_VAL_MASK            (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_VAL_SHIFT           (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_VAL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_BYP_VAL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_INV_SEL_MASK            (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_INV_SEL_SHIFT           (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_INV_SEL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_START_RAMP_INV_SEL_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_RESERVED0_MASK                     (0xFFFFFFF8U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_RESERVED0_SHIFT                    (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_RESERVED0_MAX                      (0x1FFFFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG6_SYNTH_RESETVAL                           (0x00000000U)

/* CLK_CTRL_REG4_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_0_MASK                 (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_0_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_0_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_0_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_1_MASK                 (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_1_SHIFT                (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_1_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_3_2_MASK               (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_3_2_SHIFT              (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_3_2_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_3_2_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_11_4_MASK              (0x00000FF0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_11_4_SHIFT             (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_11_4_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_11_4_MAX               (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_12_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_12_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_12_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_12_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_17_13_MASK             (0x0003E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_17_13_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_17_13_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_17_13_MAX              (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_20_18_MASK             (0x001C0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_20_18_SHIFT            (0x00000012U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_20_18_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_FB_SDM_CTRL_20_18_MAX              (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_RESERVED0_MASK                     (0xFFE00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_RESERVED0_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_RESERVED0_MAX                      (0x000007FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_SYNTH_RESETVAL                           (0x00000000U)

/* CLK_CTRL_REG5_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_VOUT_CTRL_MASK                 (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_VOUT_CTRL_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_VOUT_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_VOUT_CTRL_MAX                  (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENZ_LOW_BW_CAP_MASK                (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENZ_LOW_BW_CAP_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENZ_LOW_BW_CAP_RESETVAL            (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENZ_LOW_BW_CAP_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_TEST_MODE_MASK                  (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_TEST_MODE_SHIFT                 (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_TEST_MODE_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_TEST_MODE_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_SHRT_CKT_MASK                   (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_SHRT_CKT_SHIFT                  (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_SHRT_CKT_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_SHRT_CKT_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_BYPASS_MASK                     (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_BYPASS_SHIFT                    (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_BYPASS_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_EN_BYPASS_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_BW_CTRL_MASK                   (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_BW_CTRL_SHIFT                  (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_BW_CTRL_RESETVAL               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_LDO_BW_CTRL_MAX                    (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_SCPRT_IBIAS_CTRL_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_SCPRT_IBIAS_CTRL_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_SCPRT_IBIAS_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_SCPRT_IBIAS_CTRL_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENABLE_PMOS_PULLDOWN_MASK          (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENABLE_PMOS_PULLDOWN_SHIFT         (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENABLE_PMOS_PULLDOWN_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_ENABLE_PMOS_PULLDOWN_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TLOAD_CTRL_MASK                    (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TLOAD_CTRL_SHIFT                   (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TLOAD_CTRL_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TLOAD_CTRL_MAX                     (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TESTMUX_CTRL_MASK                  (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TESTMUX_CTRL_SHIFT                 (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TESTMUX_CTRL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_TESTMUX_CTRL_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_BISTMUX_CTRL_MASK                  (0x00F00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_BISTMUX_CTRL_SHIFT                 (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_BISTMUX_CTRL_RESETVAL              (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_BISTMUX_CTRL_MAX                   (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_RESERVED0_MASK                     (0xFF000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_RESERVED0_SHIFT                    (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_RESERVED0_MAX                      (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_SYNTH_RESETVAL                           (0x00400710U)

/* CLK_CTRL_REG4_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENZ_LOW_BW_CAP_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENZ_LOW_BW_CAP_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENZ_LOW_BW_CAP_RESETVAL       (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENZ_LOW_BW_CAP_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MASK         (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_SCPRT_IBIAS_CTRL_SHIFT        (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_SCPRT_IBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MASK     (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_SHIFT    (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TLOAD_CTRL_MASK               (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TLOAD_CTRL_SHIFT              (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TLOAD_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TLOAD_CTRL_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_APLL_MASK    (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_APLL_SHIFT   (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_APLL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_APLL_MAX     (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_APLL_MASK    (0x00F00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_APLL_SHIFT   (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_APLL_RESETVAL (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_APLL_MAX     (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_ROUTE_MASK   (0x0F000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_ROUTE_SHIFT  (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_ROUTE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_TESTMUX_CTRL_LDO_ROUTE_MAX    (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_ROUTE_MASK   (0xF0000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_ROUTE_SHIFT  (0x0000001CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_ROUTE_RESETVAL (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_BISTMUX_CTRL_LDO_ROUTE_MAX    (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_LDO_CLKTOP_RESETVAL                      (0x40400710U)

/* CLK_CTRL_REG5_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENZ_LOW_BW_CAP_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENZ_LOW_BW_CAP_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENZ_LOW_BW_CAP_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENZ_LOW_BW_CAP_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MASK         (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_SCPRT_IBIAS_CTRL_SHIFT        (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_SCPRT_IBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MASK     (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_SHIFT    (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TLOAD_CTRL_MASK               (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TLOAD_CTRL_SHIFT              (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TLOAD_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TLOAD_CTRL_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TESTMUX_CTRL_MASK             (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TESTMUX_CTRL_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_TESTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_BISTMUX_CTRL_MASK             (0x00F00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_BISTMUX_CTRL_SHIFT            (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_BISTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_BISTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_RESERVED0_MASK                (0xFF000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_RESERVED0_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_RESERVED0_MAX                 (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG5_LDO_CLKTOP_RESETVAL                      (0x00000000U)

/* CLK_CTRL_REG7_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENZ_LOW_BW_CAP_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENZ_LOW_BW_CAP_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENZ_LOW_BW_CAP_RESETVAL       (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENZ_LOW_BW_CAP_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MASK         (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_SCPRT_IBIAS_CTRL_SHIFT        (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_SCPRT_IBIAS_CTRL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MASK     (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_SHIFT    (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TLOAD_CTRL_MASK               (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TLOAD_CTRL_SHIFT              (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TLOAD_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TLOAD_CTRL_MAX                (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TESTMUX_CTRL_MASK             (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TESTMUX_CTRL_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_TESTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_BISTMUX_CTRL_MASK             (0x00F00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_BISTMUX_CTRL_SHIFT            (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_BISTMUX_CTRL_RESETVAL         (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_BISTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_RESERVED0_MASK                (0xFF000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_RESERVED0_SHIFT               (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_RESERVED0_MAX                 (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG7_LDO_CLKTOP_RESETVAL                      (0x00400710U)

/* CLK_CTRL_REG1_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_MASK                (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_RESETVAL            (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_APLL_VCO_LDO_MASK          (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_APLL_VCO_LDO_SHIFT         (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_APLL_VCO_LDO_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_APLL_VCO_LDO_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLKTOP_IOBUF_LDO_MASK      (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLKTOP_IOBUF_LDO_SHIFT     (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLKTOP_IOBUF_LDO_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLKTOP_IOBUF_LDO_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SDM_LDO_MASK               (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SDM_LDO_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SDM_LDO_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SDM_LDO_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_VCO_LDO_MASK         (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_VCO_LDO_SHIFT        (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_VCO_LDO_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_VCO_LDO_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_DIV_LDO_MASK         (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_DIV_LDO_SHIFT        (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_DIV_LDO_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_DIV_LDO_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_PFDCP_LDO_MASK       (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_PFDCP_LDO_SHIFT      (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_PFDCP_LDO_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNTH_PFDCP_LDO_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNC_20G_LDO_MASK          (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNC_20G_LDO_SHIFT         (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNC_20G_LDO_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_SYNC_20G_LDO_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLK_TOP_TEST_LDO_MASK      (0x00000100U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLK_TOP_TEST_LDO_SHIFT     (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLK_TOP_TEST_LDO_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_EN_CLK_TOP_TEST_LDO_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_MASK                (0xFFFFFE00U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_SHIFT               (0x00000009U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_MAX                 (0x007FFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_LDO_CLKTOP_RESETVAL                      (0x00000001U)

/* CLK_CTRL_REG1_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_ICP2_TRIM_MASK                     (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_ICP2_TRIM_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_ICP2_TRIM_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_ICP2_TRIM_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_I_VCO_BUF_TRIM_MASK                (0x000000F0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_I_VCO_BUF_TRIM_SHIFT               (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_I_VCO_BUF_TRIM_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_I_VCO_BUF_TRIM_MAX                 (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_UP_DLY_MASK                        (0x00000F00U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_UP_DLY_SHIFT                       (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_UP_DLY_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_UP_DLY_MAX                         (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_DN_DLY_MASK                        (0x0000F000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_DN_DLY_SHIFT                       (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_DN_DLY_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_DN_DLY_MAX                         (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO1_REP_MASK                (0x001F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO1_REP_SHIFT               (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO1_REP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO1_REP_MAX                 (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO2_REP_MASK                (0x03E00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO2_REP_SHIFT               (0x00000015U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO2_REP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RTRIM_VCO2_REP_MAX                 (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RESERVED0_MASK                     (0xFC000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RESERVED0_SHIFT                    (0x0000001AU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RESERVED0_MAX                      (0x0000003FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_SYNTH_RESETVAL                           (0x00000000U)

/* CLK_CTRL_REG2_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_TESTO_MASK                 (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_TESTO_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_TESTO_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_TESTO_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_FORCE_MASK                 (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_FORCE_SHIFT                (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_FORCE_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_VCNT_FORCE_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RING_LEN_MASK                      (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RING_LEN_SHIFT                     (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RING_LEN_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RING_LEN_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RST_TDC_MASK                       (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RST_TDC_SHIFT                      (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RST_TDC_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RST_TDC_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_100M_CLK_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_100M_CLK_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_100M_CLK_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_100M_CLK_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_10G_CLK_MASK            (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_10G_CLK_SHIFT           (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_10G_CLK_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_LIN_TDC_10G_CLK_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RESERVED_VCO1_CTRL_SWCAP_ES1P0_MASK (0x00000F80U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RESERVED_VCO1_CTRL_SWCAP_ES1P0_SHIFT (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RESERVED_VCO1_CTRL_SWCAP_ES1P0_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RESERVED_VCO1_CTRL_SWCAP_ES1P0_MAX (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DIV_PBIASCTRL_MASK           (0x0000F000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DIV_PBIASCTRL_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DIV_PBIASCTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DIV_PBIASCTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_DIV500_MASK                     (0x00010000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_DIV500_SHIFT                    (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_DIV500_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_EN_DIV500_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SEL_CLK_100M_MASK                  (0x00020000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SEL_CLK_100M_SHIFT                 (0x00000011U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SEL_CLK_100M_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SEL_CLK_100M_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO1_CTRL_SWCAP_6_0_MASK           (0x01FC0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO1_CTRL_SWCAP_6_0_SHIFT          (0x00000012U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO1_CTRL_SWCAP_6_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO1_CTRL_SWCAP_6_0_MAX            (0x0000007FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO2_CTRL_SWCAP_5_0_MASK           (0x7E000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO2_CTRL_SWCAP_5_0_SHIFT          (0x00000019U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO2_CTRL_SWCAP_5_0_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_VCO2_CTRL_SWCAP_5_0_MAX            (0x0000003FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DC_BIST_DISABLE_MASK         (0x80000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DC_BIST_DISABLE_SHIFT        (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DC_BIST_DISABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_SYNTH_DC_BIST_DISABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_SYNTH_RESETVAL                           (0x00000000U)

/* CLK_CTRL_REG3_SYNTH */

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_1_0_MASK         (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_1_0_SHIFT        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_1_0_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_1_0_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_2_MASK           (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_2_SHIFT          (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_2_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_2_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED0_MASK                     (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED0_SHIFT                    (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED0_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_4_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_4_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_4_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_4_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_5_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_5_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_5_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_CONFIG_CTRL_5_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED1_MASK                     (0x0000FFC0U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED1_SHIFT                    (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESERVED1_MAX                      (0x000003FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_VCNT_INT_FORCE_CTRL_15_0_MASK (0xFFFF0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_VCNT_INT_FORCE_CTRL_15_0_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_VCNT_INT_FORCE_CTRL_15_0_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_SYNTH_VCNT_INT_FORCE_CTRL_15_0_MAX (0x0000FFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_SYNTH_RESETVAL                           (0x00000000U)

/* CLK_CTRL_REG1_APLL */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_MDIV_APLL_MASK                      (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_MDIV_APLL_SHIFT                     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_MDIV_APLL_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_MDIV_APLL_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_MASK                      (0x000001FCU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_SHIFT                     (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_MAX                       (0x0000007FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_BURNIN_MASK               (0x00000600U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_BURNIN_SHIFT              (0x00000009U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_BURNIN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_NDIV_APLL_BURNIN_MAX                (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RTRIM_LPF_APLL_MASK                 (0x0000F800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RTRIM_LPF_APLL_SHIFT                (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RTRIM_LPF_APLL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RTRIM_LPF_APLL_MAX                  (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_CZCTL_LPF_APLL_MASK                 (0x00030000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_CZCTL_LPF_APLL_SHIFT                (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_CZCTL_LPF_APLL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_CZCTL_LPF_APLL_MAX                  (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED0_MASK                      (0x0FFC0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED0_SHIFT                     (0x00000012U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED0_MAX                       (0x000003FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_SEL_ADCCLK_APLL_MASK                (0x30000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_SEL_ADCCLK_APLL_SHIFT               (0x0000001CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_SEL_ADCCLK_APLL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_SEL_ADCCLK_APLL_MAX                 (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED1_MASK                      (0xC0000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED1_SHIFT                     (0x0000001EU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESERVED1_MAX                       (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_APLL_RESETVAL                            (0x00000000U)

/* CLK_CTRL_REG2_APLL */

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_VCO_CTRL_SWCAP_MAIN_APLL_MASK       (0x003FFFFFU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_VCO_CTRL_SWCAP_MAIN_APLL_SHIFT      (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_VCO_CTRL_SWCAP_MAIN_APLL_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_VCO_CTRL_SWCAP_MAIN_APLL_MAX        (0x003FFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_VCO_APLL_MASK                 (0x07C00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_VCO_APLL_SHIFT                (0x00000016U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_VCO_APLL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_VCO_APLL_MAX                  (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_REP_APLL_MASK                 (0xF8000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_REP_APLL_SHIFT                (0x0000001BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_REP_APLL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RTRIM_REP_APLL_MAX                  (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_APLL_RESETVAL                            (0x00000000U)

/* CLK_CTRL_REG3_APLL */

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_TST_BUFEN_APLL_MASK                 (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_TST_BUFEN_APLL_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_TST_BUFEN_APLL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_TST_BUFEN_APLL_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCONT_MUXEN_APLL_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCONT_MUXEN_APLL_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCONT_MUXEN_APLL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCONT_MUXEN_APLL_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_EN_MASK                 (0x00000004U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_EN_SHIFT                (0x00000002U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_EN_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_EN_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_CTRL_MASK               (0x00007FF8U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_CTRL_SHIFT              (0x00000003U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_CTRL_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_VCO_CAL_DAC_CTRL_MAX                (0x00000FFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED0_MASK                      (0x00008000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED0_SHIFT                     (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED0_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_SEL_HSICLK_APLL_MASK                (0x00030000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_SEL_HSICLK_APLL_SHIFT               (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_SEL_HSICLK_APLL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_SEL_HSICLK_APLL_MAX                 (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DIV_HSICLK_APLL_MASK                (0x000C0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DIV_HSICLK_APLL_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DIV_HSICLK_APLL_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DIV_HSICLK_APLL_MAX                 (0x00000003U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED1_MASK                      (0x7FF00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED1_SHIFT                     (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESERVED1_MAX                       (0x000007FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DC_BIST_APLL_MASK                   (0x80000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DC_BIST_APLL_SHIFT                  (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DC_BIST_APLL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_DC_BIST_APLL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_APLL_RESETVAL                            (0x00000000U)

/* CLK_CTRL_REG4_APLL */

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL_RESERVED0_MASK                      (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL_RESERVED0_SHIFT                     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL_RESERVED0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL_RESERVED0_MAX                       (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG4_APLL_RESETVAL                            (0x00000000U)

/* CLK_CTRL_REG1_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED0_MASK                    (0x000000FFU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED0_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED0_MAX                     (0x000000FFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_APLL_MASK                  (0x00000100U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_APLL_SHIFT                 (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_APLL_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_APLL_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_HSI_MASK               (0x00000200U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_HSI_SHIFT              (0x00000009U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_HSI_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_HSI_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1200_DIG_MASK           (0x00000400U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1200_DIG_SHIFT          (0x0000000AU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1200_DIG_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1200_DIG_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_HSI_MASK              (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_HSI_SHIFT             (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_HSI_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_HSI_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_SYNTHADC_MASK         (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_SYNTHADC_SHIFT        (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_SYNTHADC_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_TREE_SYNTHADC_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_SYNTH_MASK             (0x00002000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_SYNTH_SHIFT            (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_SYNTH_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_SYNTH_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_ADC_MASK               (0x00004000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_ADC_SHIFT              (0x0000000EU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_ADC_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK_ADC_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESET_APLL_MASK                   (0x00008000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESET_APLL_SHIFT                  (0x0000000FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESET_APLL_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESET_APLL_MAX                    (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_MASK                           (0x00010000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_SHIFT                          (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_PLL_RESET_MASK                    (0x00020000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_PLL_RESET_SHIFT                   (0x00000011U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_PLL_RESET_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_PLL_RESET_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO1_MASK                      (0x00040000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO1_SHIFT                     (0x00000012U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO1_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO1_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO2_MASK                      (0x00080000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO2_SHIFT                     (0x00000013U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO2_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_VCO2_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_PFDCP_MASK                     (0x00100000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_PFDCP_SHIFT                    (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_PFDCP_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_PFDCP_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED1_MASK                    (0x00200000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED1_SHIFT                   (0x00000015U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED1_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED1_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_TEST_DIV_MASK                  (0x00400000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_TEST_DIV_SHIFT                 (0x00000016U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_TEST_DIV_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_EN_TEST_DIV_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1800_DIG_MASK           (0x00800000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1800_DIG_SHIFT          (0x00000017U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1800_DIG_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ENABLE_CLK1800_DIG_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ISOZ_DIG_CLK_PATH_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ISOZ_DIG_CLK_PATH_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ISOZ_DIG_CLK_PATH_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_ISOZ_DIG_CLK_PATH_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED2_MASK                    (0xFE000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED2_SHIFT                   (0x00000019U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED2_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESERVED2_MAX                     (0x0000007FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_CLKTOP_RESETVAL                          (0x00000000U)

/* CLK_CTRL_REG3_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_VOUT_CTRL_MASK            (0x0000001FU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_VOUT_CTRL_MAX             (0x0000001FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_TEST_MODE_MASK             (0x00000020U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_TEST_MODE_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_TEST_MODE_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_TEST_MODE_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_SHRT_CKT_MASK              (0x00000040U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_SHRT_CKT_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_SHRT_CKT_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_BYPASS_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_BYPASS_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_BYPASS_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_EN_BYPASS_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_BW_CTRL_MASK              (0x00000700U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_BW_CTRL_SHIFT             (0x00000008U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_LDO_BW_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED1_MASK                (0x00000800U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED1_SHIFT               (0x0000000BU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED1_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED1_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED2_MASK                (0x00001000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED2_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED2_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED2_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED3_MASK                (0x0000E000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED3_SHIFT               (0x0000000DU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED3_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESERVED3_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_TESTMUX_CTRL_MASK             (0x000F0000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_TESTMUX_CTRL_SHIFT            (0x00000010U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_TESTMUX_CTRL_MAX              (0x0000000FU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_BISTMUX_CTRL_MASK             (0xFFF00000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_BISTMUX_CTRL_SHIFT            (0x00000014U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_BISTMUX_CTRL_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_BISTMUX_CTRL_MAX              (0x00000FFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG3_LDO_CLKTOP_RESETVAL                      (0x00000000U)

/* CLK_CTRL_REG2_LDO_CLKTOP */

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_MASK                (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_MAX                 (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG2_LDO_CLKTOP_RESETVAL                      (0x00000000U)

/* CLK_CTRL_REG1_XO_SLICER */

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER_RESERVED0_MASK                 (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER_RESERVED0_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER_RESERVED0_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER_RESERVED0_MAX                  (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_CLK_CTRL_REG1_XO_SLICER_RESETVAL                       (0x00000000U)

/* EFUSE_SPARE_REG */

#define CSL_AR_RFANACIO_EFUSE_SPARE_REG_RESERVED0_MASK                         (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_EFUSE_SPARE_REG_RESERVED0_SHIFT                        (0x00000000U)
#define CSL_AR_RFANACIO_EFUSE_SPARE_REG_RESERVED0_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_EFUSE_SPARE_REG_RESERVED0_MAX                          (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_EFUSE_SPARE_REG_RESETVAL                               (0x00000000U)

/* ODP_BIST_CTRL */

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_0_MASK                       (0x00000001U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_0_SHIFT                      (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_0_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_1_MASK                       (0x00000002U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_1_SHIFT                      (0x00000001U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_1_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_2_MASK                       (0x00000004U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_2_SHIFT                      (0x00000002U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_2_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_2_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_3_MASK                       (0x00000008U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_3_SHIFT                      (0x00000003U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_3_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_3_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_4_MASK                       (0x00000010U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_4_SHIFT                      (0x00000004U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_4_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_4_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_5_MASK                       (0x00000020U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_5_SHIFT                      (0x00000005U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_5_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_5_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_6_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_6_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_6_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_6_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_7_MASK                       (0x00000080U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_7_SHIFT                      (0x00000007U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_7_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_7_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_8_MASK                       (0x00000100U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_8_SHIFT                      (0x00000008U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_8_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_8_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_9_MASK                       (0x00000200U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_9_SHIFT                      (0x00000009U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_9_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_9_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_10_MASK                      (0x00000400U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_10_SHIFT                     (0x0000000AU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_10_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_10_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_11_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_11_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_11_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_11_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_RESERVED0_MASK                           (0x00001000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_RESERVED0_SHIFT                          (0x0000000CU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_RESERVED0_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_13_MASK                      (0x00002000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_13_SHIFT                     (0x0000000DU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_13_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_13_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_14_MASK                      (0x00004000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_14_SHIFT                     (0x0000000EU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_14_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_14_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_15_MASK                      (0x00008000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_15_SHIFT                     (0x0000000FU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_15_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_DC_15_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_0_MASK                       (0x00010000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_0_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_0_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_1_MASK                       (0x00020000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_1_SHIFT                      (0x00000011U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_1_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_2_MASK                       (0x00040000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_2_SHIFT                      (0x00000012U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_2_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_2_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_3_MASK                       (0x00080000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_3_SHIFT                      (0x00000013U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_3_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_3_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_4_MASK                       (0x00100000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_4_SHIFT                      (0x00000014U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_4_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_4_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_5_MASK                       (0x00200000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_5_SHIFT                      (0x00000015U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_5_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_5_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_6_MASK                       (0x00400000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_6_SHIFT                      (0x00000016U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_6_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_6_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_7_MASK                       (0x00800000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_7_SHIFT                      (0x00000017U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_7_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_7_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_8_MASK                       (0x01000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_8_SHIFT                      (0x00000018U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_8_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_8_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_9_MASK                       (0x02000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_9_SHIFT                      (0x00000019U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_9_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_9_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_10_MASK                      (0x04000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_10_SHIFT                     (0x0000001AU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_10_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_10_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_11_MASK                      (0x08000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_11_SHIFT                     (0x0000001BU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_11_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_11_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_12_MASK                      (0x10000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_12_SHIFT                     (0x0000001CU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_12_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_12_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_13_MASK                      (0x20000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_13_SHIFT                     (0x0000001DU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_13_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_13_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_14_MASK                      (0x40000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_14_SHIFT                     (0x0000001EU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_14_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_14_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_15_MASK                      (0x80000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_15_SHIFT                     (0x0000001FU)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_15_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_ODP_BIST_CTRL_CTRL_ODP_AC_15_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_ODP_BIST_CTRL_RESETVAL                                 (0x00000000U)

/* REFSYS_SPARE_REG2 */

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_RSET_EN_MASK              (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_RSET_EN_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_RSET_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_RSET_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_RSET_EN_MASK               (0x00000002U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_RSET_EN_SHIFT              (0x00000001U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_RSET_EN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_RSET_EN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_RSET_EN_MASK            (0x00000004U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_RSET_EN_SHIFT           (0x00000002U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_RSET_EN_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_RSET_EN_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_RSET_EN_MASK            (0x00000008U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_RSET_EN_SHIFT           (0x00000003U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_RSET_EN_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_RSET_EN_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED_MASK                        (0x000003F0U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED_SHIFT                       (0x00000004U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED_MAX                         (0x0000003FU)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_SELF_TEST_EN_MASK         (0x00000400U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_SELF_TEST_EN_SHIFT        (0x0000000AU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_SELF_TEST_EN_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_SELF_TEST_EN_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_SELF_TEST_EN_MASK          (0x00000800U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_SELF_TEST_EN_SHIFT         (0x0000000BU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_SELF_TEST_EN_RESETVAL      (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_SELF_TEST_EN_MAX           (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_SELF_TEST_EN_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_SELF_TEST_EN_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_SELF_TEST_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_SELF_TEST_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_SELF_TEST_EN_MASK       (0x00002000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_SELF_TEST_EN_SHIFT      (0x0000000DU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_SELF_TEST_EN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_SELF_TEST_EN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED1_MASK                       (0x00004000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED1_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED1_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED2_MASK                       (0x00008000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED2_SHIFT                      (0x0000000FU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED2_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED2_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED3_MASK                       (0x00010000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED3_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED3_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED3_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED4_MASK                       (0x00020000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED4_SHIFT                      (0x00000011U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED4_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED4_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED5_MASK                       (0x000C0000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED5_SHIFT                      (0x00000012U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED5_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESERVED5_MAX                        (0x00000003U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_IR_DROP_COMP_SEL_MASK        (0x00300000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_IR_DROP_COMP_SEL_SHIFT       (0x00000014U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_IR_DROP_COMP_SEL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_IR_DROP_COMP_SEL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_IR_DROP_COMP_SEL_MASK         (0x00C00000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_IR_DROP_COMP_SEL_SHIFT        (0x00000016U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_IR_DROP_COMP_SEL_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_IR_DROP_COMP_SEL_MAX          (0x00000003U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_IR_DROP_COMP_SEL_MASK      (0x03000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_IR_DROP_COMP_SEL_SHIFT     (0x00000018U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_IR_DROP_COMP_SEL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_IR_DROP_COMP_SEL_MAX       (0x00000003U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_IR_DROP_COMP_SEL_MASK      (0x0C000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_IR_DROP_COMP_SEL_SHIFT     (0x0000001AU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_IR_DROP_COMP_SEL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_IR_DROP_COMP_SEL_MAX       (0x00000003U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_VMON_EN_MASK              (0x10000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_VMON_EN_SHIFT             (0x0000001CU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_VMON_EN_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDAVCO_UV_VMON_EN_MAX               (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_VMON_EN_MASK               (0x20000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_VMON_EN_SHIFT              (0x0000001DU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_VMON_EN_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDABB_UV_VMON_EN_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_VMON_EN_MASK            (0x40000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_VMON_EN_SHIFT           (0x0000001EU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_VMON_EN_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF1_UV_VMON_EN_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_VMON_EN_MASK            (0x80000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_VMON_EN_SHIFT           (0x0000001FU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_VMON_EN_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_VDDA10RF2_UV_VMON_EN_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG2_RESETVAL                             (0x00000000U)

/* REFSYS_SPARE_REG3 */

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3_RESERVED_MASK                        (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3_RESERVED_SHIFT                       (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3_RESERVED_MAX                         (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_REFSYS_SPARE_REG3_RESETVAL                             (0x00000000U)

/* RX_TOP_SPARE_REG3 */

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3_RX_TOP_SPARE_REG3_MASK               (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3_RX_TOP_SPARE_REG3_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3_RX_TOP_SPARE_REG3_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3_RX_TOP_SPARE_REG3_MAX                (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG3_RESETVAL                             (0x00000000U)

/* RX_TOP_SPARE_REG4 */

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4_RX_TOP_SPARE_REG4_MASK               (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4_RX_TOP_SPARE_REG4_SHIFT              (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4_RX_TOP_SPARE_REG4_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4_RX_TOP_SPARE_REG4_MAX                (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RX_TOP_SPARE_REG4_RESETVAL                             (0x00000000U)

/* TX_PWRSW_CTRL */

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_IQGEN_PS_PWR_SWITCH_CTRL_MASK        (0x00000003U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_IQGEN_PS_PWR_SWITCH_CTRL_SHIFT       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_IQGEN_PS_PWR_SWITCH_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_IQGEN_PS_PWR_SWITCH_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_PA_PWR_SWITCH_CTRL_MASK              (0x0000001CU)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_PA_PWR_SWITCH_CTRL_SHIFT             (0x00000002U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_PA_PWR_SWITCH_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX1_PA_PWR_SWITCH_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED0_MASK                           (0x000000E0U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED0_SHIFT                          (0x00000005U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED0_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_IQGEN_PS_PWR_SWITCH_CTRL_MASK        (0x00000300U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_IQGEN_PS_PWR_SWITCH_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_IQGEN_PS_PWR_SWITCH_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_IQGEN_PS_PWR_SWITCH_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_PA_PWR_SWITCH_CTRL_MASK              (0x00001C00U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_PA_PWR_SWITCH_CTRL_SHIFT             (0x0000000AU)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_PA_PWR_SWITCH_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX2_PA_PWR_SWITCH_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED1_MASK                           (0x0000E000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED1_SHIFT                          (0x0000000DU)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED1_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED1_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_IQGEN_PS_PWR_SWITCH_CTRL_MASK        (0x00030000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_IQGEN_PS_PWR_SWITCH_CTRL_SHIFT       (0x00000010U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_IQGEN_PS_PWR_SWITCH_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_IQGEN_PS_PWR_SWITCH_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_PA_PWR_SWITCH_CTRL_MASK              (0x001C0000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_PA_PWR_SWITCH_CTRL_SHIFT             (0x00000012U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_PA_PWR_SWITCH_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX3_PA_PWR_SWITCH_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED2_MASK                           (0x00E00000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED2_SHIFT                          (0x00000015U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED2_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED2_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_IQGEN_PS_PWR_SWITCH_CTRL_MASK        (0x03000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_IQGEN_PS_PWR_SWITCH_CTRL_SHIFT       (0x00000018U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_IQGEN_PS_PWR_SWITCH_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_IQGEN_PS_PWR_SWITCH_CTRL_MAX         (0x00000003U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_PA_PWR_SWITCH_CTRL_MASK              (0x1C000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_PA_PWR_SWITCH_CTRL_SHIFT             (0x0000001AU)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_PA_PWR_SWITCH_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_TX4_PA_PWR_SWITCH_CTRL_MAX               (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED3_MASK                           (0xE0000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED3_SHIFT                          (0x0000001DU)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED3_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESERVED3_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX_PWRSW_CTRL_RESETVAL                                 (0x00000000U)

/* TX4_BIST_LNA_CTRL */

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MASK              (0x0000001FU)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_GAIN_CTRL_MAX               (0x0000001FU)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MASK             (0x000003E0U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_SHIFT            (0x00000005U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_PD_ENABLES_MAX              (0x0000001FU)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MASK      (0x00000400U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_SHIFT     (0x0000000AU)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MASK        (0x0001F800U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_SHIFT       (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_CTRL_MAX         (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MASK        (0x00020000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_SHIFT       (0x00000011U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_OFFSET_DAC_SIGN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MASK     (0x00040000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_SHIFT    (0x00000012U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_CALIBRATION_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MASK       (0x00380000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_SHIFT      (0x00000013U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_BANDWIDTH_CONFIG_MAX        (0x00000007U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MASK     (0x00C00000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_SHIFT    (0x00000016U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_COMMON_MODE_CONFIG_MAX      (0x00000003U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MASK            (0x01000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_SHIFT           (0x00000018U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_RESETVAL        (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_BIST_LNA_VOUT_ENABLE_MAX             (0x00000001U)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_RESERVED0_MASK                       (0xFE000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_RESERVED0_SHIFT                      (0x00000019U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_RESERVED0_MAX                        (0x0000007FU)

#define CSL_AR_RFANACIO_TX4_BIST_LNA_CTRL_RESETVAL                             (0x00000000U)

/* TX4_CTRL_RC_FILT */

#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MASK           (0x03U)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_SHIFT          (0x00U)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_RESETVAL       (0x00U)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_PS_CTRL_RC_FILT_BW_SEL_MAX            (0x03U)

#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_RESERVED0_MASK                        (0x0CU)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_RESERVED0_SHIFT                       (0x02U)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_RESERVED0_RESETVAL                    (0x00U)
#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_RESERVED0_MAX                         (0x03U)

#define CSL_AR_RFANACIO_TX4_CTRL_RC_FILT_RESETVAL                              (0x00U)

/* TX4_DAC_ANALDO_CTRL */

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MASK           (0x0000001EU)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_CTRL_MAX            (0x0000000FU)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MASK     (0x00000020U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_SHIFT    (0x00000005U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_LOW_BW_ENZ_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MASK        (0x00000700U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_BW_CTRL_MAX         (0x00000007U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MASK (0x00000800U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_SCPRT_IBIAS_CTRL_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_PMOS_PULLDWN_EN_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MASK     (0x0000E000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_SHIFT    (0x0000000DU)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TLOAD_CTRL_MAX      (0x00000007U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MASK (0x00010000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_SHIFT (0x00000010U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VOUT_SENSE_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MASK (0x00020000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_SHIFT (0x00000011U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VIN_SENSE_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MASK      (0x00040000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_SHIFT     (0x00000012U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_VSSA_TEST_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_ITEST_12P5UA_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MASK        (0x00100000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_SHIFT       (0x00000014U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_TX_ANA_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_ANALDO_CTRL_RESETVAL                           (0x00000000U)

/* ANA_DFE_ISO_CTRL */

#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL_ANA_DFE_ISO_CTRL_MASK                 (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL_ANA_DFE_ISO_CTRL_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL_ANA_DFE_ISO_CTRL_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL_ANA_DFE_ISO_CTRL_MAX                  (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_ANA_DFE_ISO_CTRL_RESETVAL                              (0x00000000U)

/* TX4_DAC_CONFIG */

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_EN_MASK                            (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_EN_SHIFT                           (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_EN_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_EN_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_CTRL_MASK                          (0x0000000EU)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_CTRL_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_CTRL_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_BIAS_CTRL_MAX                           (0x00000007U)

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_ICNTRL_MASK                         (0x00000FF0U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_ICNTRL_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_ICNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_ICNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_QCNTRL_MASK                         (0x000FF000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_QCNTRL_SHIFT                        (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_QCNTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_DAC_QCNTRL_MAX                          (0x000000FFU)

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_RESERVED_MASK                           (0xFFF00000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_RESERVED_SHIFT                          (0x00000014U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_RESERVED_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_RESERVED_MAX                            (0x00000FFFU)

#define CSL_AR_RFANACIO_TX4_DAC_CONFIG_RESETVAL                                (0x00000000U)

/* TX4_DAC_CTRL */

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKDLY_MASK                           (0x00000007U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKDLY_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKINVEN_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKINVEN_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ILATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLKINV_MASK                              (0x00000010U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLKINV_SHIFT                             (0x00000004U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_ICAL_MASK                              (0x00000020U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_ICAL_SHIFT                             (0x00000005U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_ICAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_ICAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICALCLKSEL_MASK                           (0x000003C0U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICALCLKSEL_SHIFT                          (0x00000006U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_IDEC_MASK                              (0x00000400U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_IDEC_SHIFT                             (0x0000000AU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_IDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_IDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACI_RESETB_MASK                          (0x00000800U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACI_RESETB_SHIFT                         (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACI_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACI_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVI_DEC_MASK                          (0x00001000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVI_DEC_SHIFT                         (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVI_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVI_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLK_DEC_EN_MASK                          (0x00002000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLK_DEC_EN_SHIFT                         (0x0000000DU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ICLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ISIGN_EN_MASK                             (0x00004000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ISIGN_EN_SHIFT                            (0x0000000EU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ISIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_ISIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKDLY_MASK                           (0x00038000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKDLY_SHIFT                          (0x0000000FU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKDLY_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKDLY_MAX                            (0x00000007U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKINVEN_MASK                         (0x00040000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKINVEN_SHIFT                        (0x00000012U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKINVEN_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QLATCLKINVEN_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLKINV_MASK                              (0x00080000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLKINV_SHIFT                             (0x00000013U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLKINV_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLKINV_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QCAL_MASK                              (0x00100000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QCAL_SHIFT                             (0x00000014U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QCAL_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QCAL_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCALCLKSEL_MASK                           (0x01E00000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCALCLKSEL_SHIFT                          (0x00000015U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCALCLKSEL_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCALCLKSEL_MAX                            (0x0000000FU)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QDEC_MASK                              (0x02000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QDEC_SHIFT                             (0x00000019U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QDEC_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_EN_QDEC_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACQ_RESETB_MASK                          (0x04000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACQ_RESETB_SHIFT                         (0x0000001AU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACQ_RESETB_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_DACQ_RESETB_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVQ_DEC_MASK                          (0x08000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVQ_DEC_SHIFT                         (0x0000001BU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVQ_DEC_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_CLKDIVQ_DEC_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLK_DEC_EN_MASK                          (0x10000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLK_DEC_EN_SHIFT                         (0x0000001CU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLK_DEC_EN_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QCLK_DEC_EN_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QSIGN_EN_MASK                             (0x20000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QSIGN_EN_SHIFT                            (0x0000001DU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QSIGN_EN_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_QSIGN_EN_MAX                              (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_RESERVED_MASK                             (0xC0000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_RESERVED_SHIFT                            (0x0000001EU)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_CTRL_RESERVED_MAX                              (0x00000003U)

#define CSL_AR_RFANACIO_TX4_DAC_CTRL_RESETVAL                                  (0x00000000U)

/* TX4_DAC_DIGLDO_CTRL */

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MASK             (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_SHIFT            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_RESETVAL         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_EN_MAX              (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MASK           (0x0000003EU)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_CTRL_MAX            (0x0000001FU)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MASK    (0x00000040U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_SHIFT   (0x00000006U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_SHRT_CKT_EN_MAX     (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_BYPASS_EN_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MASK        (0x00000100U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_SHIFT       (0x00000008U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_TEST_EN_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MASK (0x00000200U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_SHIFT (0x00000009U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDDA_1P8V_TEST_MAX  (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VDD_DEC_1P1V_TEST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MASK       (0x00000800U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_SHIFT      (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VSS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MASK       (0x00001000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_SHIFT      (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DIG_DAC_LDO_VREF_0P9_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MASK   (0x001FE000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_SHIFT  (0x0000000DU)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_TX_DAC_LDO_BIST_BYPASS_CTRL_MAX    (0x000000FFU)

#define CSL_AR_RFANACIO_TX4_DAC_DIGLDO_CTRL_RESETVAL                           (0x00000000U)

/* TX4_IQGEN_BIAS_CTRL */

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MASK              (0x0000003FU)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_SHIFT             (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_I_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MASK              (0x00000FC0U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_SHIFT             (0x00000006U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_RESETVAL          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_TX_IQGEN_Q_BUFF1_MAX               (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED0_MASK                     (0x0003F000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED0_SHIFT                    (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED0_MAX                      (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_LODIST_TX4_AMP_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_LODIST_TX4_AMP_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_LODIST_TX4_AMP_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_LODIST_TX4_AMP_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED1_MASK                     (0x3F000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED1_SHIFT                    (0x00000018U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED1_MAX                      (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED2_MASK                     (0x40000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED2_SHIFT                    (0x0000001EU)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED2_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED2_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED3_MASK                     (0x80000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED3_SHIFT                    (0x0000001FU)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED3_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESERVED3_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGEN_BIAS_CTRL_RESETVAL                           (0x00000000U)

/* TX4_IQGENLDO_CTRL */

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MASK         (0x00000001U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_SHIFT        (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_ENABLE_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MASK           (0x000000FEU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_SHIFT          (0x00000001U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_IBIAS_PD_CTRL_MAX            (0x0000007FU)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MASK     (0x00000100U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_SHIFT    (0x00000008U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MASK      (0x00000200U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_SHIFT     (0x00000009U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VIN_ANATEST_ENABLE_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MASK (0x00000400U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_SHIFT (0x0000000AU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VOUT_STG1_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MASK     (0x00000800U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_SHIFT    (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_VREF_ANATEST_ENABLE_MAX      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MASK (0x00001000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_ITEST_100UA_ANATEST_ENABLE_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MASK (0x00002000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_SHIFT (0x0000000DU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_LNADAC1_BIAS_BLOCK_CURRENT_BOOST_MAX (0x00000001U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED0_MASK                       (0x0000C000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED0_SHIFT                      (0x0000000EU)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED0_MAX                        (0x00000003U)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED1_MASK                       (0xFFFF0000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED1_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESERVED1_MAX                        (0x0000FFFFU)

#define CSL_AR_RFANACIO_TX4_IQGENLDO_CTRL_RESETVAL                             (0x00000000U)

/* TX4_PA_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG1_MASK                 (0x0000003FU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG1_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG1_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG1_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG2_MASK                 (0x00000FC0U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG2_SHIFT                (0x00000006U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG2_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG2_MAX                  (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MASK                (0x0003F000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3L_SHIFT               (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MASK                (0x00FC0000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3R_SHIFT               (0x00000012U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_BIAS_STG3R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_RESERVED0_MASK                    (0x1F000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_RESERVED0_SHIFT                   (0x00000018U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_RESERVED0_MAX                     (0x0000001FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MASK                (0xE0000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_IBIAS_TEST_SHIFT               (0x0000001DU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_IBIAS_TEST_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_PA_IBIAS_TEST_MAX                 (0x00000007U)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX4_PA_BIASCTRL_REG2 */

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MASK                (0x0000003FU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4L_SHIFT               (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4L_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4L_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MASK                (0x00000FC0U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4R_SHIFT               (0x00000006U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4R_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_PA_BIAS_STG4R_MAX                 (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_RESERVED0_MASK                    (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_RESERVED0_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_RESERVED0_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_RESERVED0_MAX                     (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX4_PA_BIASCTRL_REG2_RESETVAL                          (0x00000000U)

/* TX4_PA_EN */

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG1_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG1_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG1_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG2_MASK                          (0x00000002U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG2_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG2_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3L_MASK                         (0x00000004U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3L_SHIFT                        (0x00000002U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3R_MASK                         (0x00000008U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3R_SHIFT                        (0x00000003U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG3R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4L_MASK                         (0x00000010U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4L_SHIFT                        (0x00000004U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4L_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4L_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4R_MASK                         (0x00000020U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4R_SHIFT                        (0x00000005U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4R_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_STG4R_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_BIAS_MASTER_EN_MASK                       (0x00000040U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_BIAS_MASTER_EN_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_BIAS_MASTER_EN_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_BIAS_MASTER_EN_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_CTRL_PATH_MASK                     (0x00000080U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_CTRL_PATH_SHIFT                    (0x00000007U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_CTRL_PATH_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_PA_ENABLE_CTRL_PATH_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED0_MASK                               (0x00000F00U)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED0_SHIFT                              (0x00000008U)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED0_MAX                                (0x0000000FU)

#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED1_MASK                               (0xFFFFF000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED1_SHIFT                              (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_EN_RESERVED1_MAX                                (0x000FFFFFU)

#define CSL_AR_RFANACIO_TX4_PA_EN_RESETVAL                                     (0x00000000U)

/* TX4_PA_SPARES */

#define CSL_AR_RFANACIO_TX4_PA_SPARES_RESERVED0_MASK                           (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_TX4_PA_SPARES_RESERVED0_SHIFT                          (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_SPARES_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_SPARES_RESERVED0_MAX                            (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_TX4_PA_SPARES_RESETVAL                                 (0x00000000U)

/* TX4_PA_TESTMUX_CTRL */

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED1_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED1_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED1_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED1_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MASK         (0x00000002U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_SHIFT        (0x00000001U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MASK       (0x00000004U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_SHIFT      (0x00000002U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MASK       (0x00000008U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_SHIFT      (0x00000003U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG2DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MASK       (0x00000010U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_SHIFT      (0x00000004U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MASK       (0x00000020U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_SHIFT      (0x00000005U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MASK      (0x00000040U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_SHIFT     (0x00000006U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG4UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MASK      (0x00000080U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_SHIFT     (0x00000007U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VDD_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MASK         (0x00000100U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_SHIFT        (0x00000008U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MASK       (0x00000200U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_SHIFT      (0x00000009U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MASK       (0x00000400U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_SHIFT      (0x0000000AU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MASK      (0x00000800U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_SHIFT     (0x0000000BU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5DNL_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MASK      (0x00001000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_SHIFT     (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VSS_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MASK         (0x00002000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_SHIFT        (0x0000000DU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_RESETVAL     (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG1_MAX          (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MASK       (0x00004000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_SHIFT      (0x0000000EU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG2UP_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MASK       (0x00008000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_SHIFT      (0x0000000FU)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG3DN_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MASK      (0x00010000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_SHIFT     (0x00000010U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG4DNR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MASK      (0x00020000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_SHIFT     (0x00000011U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_VGG_STG5UPR_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MASK        (0x00040000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_SHIFT       (0x00000012U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_RESETVAL    (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_100U_MAX         (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MASK   (0x00080000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_SHIFT  (0x00000013U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IREF_CASC_100U_MAX    (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MASK       (0x00100000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_SHIFT      (0x00000014U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_PA_TMUX_CTRL_IBIAS_TEST_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED0_MASK                     (0x01E00000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED0_SHIFT                    (0x00000015U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED0_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESERVED0_MAX                      (0x0000000FU)

#define CSL_AR_RFANACIO_TX4_PA_TESTMUX_CTRL_RESETVAL                           (0x00000000U)

/* TX4_PS_BIASCTRL_REG1 */

#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MASK (0x0000003FU)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_SHIFT (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_OUTPUT_BUFFER_BIAS_CTRL_TX_MAX (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MASK (0x00000FC0U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_SHIFT (0x00000006U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_I_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MASK (0x0003F000U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_SHIFT (0x0000000CU)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_RESETVAL (0x00000000U)
#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_VM_LOIN_GATE_Q_BIAS_CNTRL_TX_MAX  (0x0000003FU)

#define CSL_AR_RFANACIO_TX4_PS_BIASCTRL_REG1_RESETVAL                          (0x00000000U)

/* TX4_PS_TMUXCTRL */

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MASK              (0x0001U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_SHIFT             (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MASK              (0x0002U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_SHIFT             (0x0001U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_IM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MASK              (0x0004U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_SHIFT             (0x0002U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QP_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MASK              (0x0008U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_SHIFT             (0x0003U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_RESETVAL          (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PS_DAC_IO_TMUX_QM_EN_MAX               (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MASK                 (0x0010U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_SHIFT                (0x0004U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DC_TEST_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MASK             (0x0020U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_SHIFT            (0x0005U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MASK             (0x0040U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_SHIFT            (0x0006U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG1_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MASK             (0x0080U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_SHIFT            (0x0007U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOI_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MASK             (0x0100U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_SHIFT            (0x0008U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_RESETVAL         (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOQ_STG2_MAX              (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MASK               (0x0200U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_SHIFT              (0x0009U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOI_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MASK               (0x0400U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_SHIFT              (0x000AU)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_RESETVAL           (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_LOQ_MAX                (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MASK            (0x0800U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_SHIFT           (0x000BU)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_RESETVAL        (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_VM_OUTBUF_MAX             (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MASK         (0x1000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_SHIFT        (0x000CU)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_RESETVAL     (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSIQGN_DCTST_LOOPBACK_BUF_MAX          (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MASK                 (0x2000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_SHIFT                (0x000DU)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_RESETVAL             (0x0000U)
#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_PSLDO_TEST_MUX_EN_MAX                  (0x0001U)

#define CSL_AR_RFANACIO_TX4_PS_TMUXCTRL_RESETVAL                               (0x0000U)

/* TXPALOOPBCKCLK */

#define CSL_AR_RFANACIO_TXPALOOPBCKCLK_TXPALOOPBCKCLK_MASK                     (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_TXPALOOPBCKCLK_TXPALOOPBCKCLK_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TXPALOOPBCKCLK_TXPALOOPBCKCLK_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TXPALOOPBCKCLK_TXPALOOPBCKCLK_MAX                      (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_TXPALOOPBCKCLK_RESETVAL                                (0x00000000U)

/* RXIFALOOPBCKCLK */

#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK_RXIFALOOPBCKCLK_MASK                   (0xFFFFFFFFU)
#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK_RXIFALOOPBCKCLK_SHIFT                  (0x00000000U)
#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK_RXIFALOOPBCKCLK_RESETVAL               (0x00000000U)
#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK_RXIFALOOPBCKCLK_MAX                    (0xFFFFFFFFU)

#define CSL_AR_RFANACIO_RXIFALOOPBCKCLK_RESETVAL                               (0x00000000U)

/* RX_STATUS_REG1 */

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED0_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED0_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED0_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED0_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED1_MASK                          (0x00000002U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED1_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED1_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_IFA_I_LDO_STATUS_MASK               (0x00000004U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_IFA_I_LDO_STATUS_SHIFT              (0x00000002U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_IFA_I_LDO_STATUS_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_IFA_I_LDO_STATUS_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED2_MASK                          (0x00000008U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED2_SHIFT                         (0x00000003U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED2_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_ANA_LDO_STATUS_MASK           (0x00000010U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_ANA_LDO_STATUS_SHIFT          (0x00000004U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_ANA_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_ANA_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_DIG_LDO_STATUS_MASK           (0x00000020U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_DIG_LDO_STATUS_SHIFT          (0x00000005U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_DIG_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX1_ADC_I_DIG_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED3_MASK                          (0x00000040U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED3_SHIFT                         (0x00000006U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED3_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED3_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED4_MASK                          (0x00000080U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED4_SHIFT                         (0x00000007U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED4_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED4_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED5_MASK                          (0x00000100U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED5_SHIFT                         (0x00000008U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED5_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED5_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED6_MASK                          (0x00000200U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED6_SHIFT                         (0x00000009U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED6_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED6_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_IFA_I_LDO_STATUS_MASK               (0x00000400U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_IFA_I_LDO_STATUS_SHIFT              (0x0000000AU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_IFA_I_LDO_STATUS_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_IFA_I_LDO_STATUS_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED7_MASK                          (0x00000800U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED7_SHIFT                         (0x0000000BU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED7_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED7_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_ANA_LDO_STATUS_MASK           (0x00001000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_ANA_LDO_STATUS_SHIFT          (0x0000000CU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_ANA_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_ANA_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_DIG_LDO_STATUS_MASK           (0x00002000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_DIG_LDO_STATUS_SHIFT          (0x0000000DU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_DIG_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX2_ADC_I_DIG_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED8_MASK                          (0x00004000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED8_SHIFT                         (0x0000000EU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED8_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED8_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED9_MASK                          (0x00008000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED9_SHIFT                         (0x0000000FU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED9_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED9_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED10_MASK                         (0x00010000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED10_SHIFT                        (0x00000010U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED10_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED10_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED11_MASK                         (0x00020000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED11_SHIFT                        (0x00000011U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED11_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED11_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_IFA_I_LDO_STATUS_MASK               (0x00040000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_IFA_I_LDO_STATUS_SHIFT              (0x00000012U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_IFA_I_LDO_STATUS_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_IFA_I_LDO_STATUS_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED12_MASK                         (0x00080000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED12_SHIFT                        (0x00000013U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED12_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED12_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_ANA_LDO_STATUS_MASK           (0x00100000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_ANA_LDO_STATUS_SHIFT          (0x00000014U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_ANA_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_ANA_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_DIG_LDO_STATUS_MASK           (0x00200000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_DIG_LDO_STATUS_SHIFT          (0x00000015U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_DIG_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX3_ADC_I_DIG_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED13_MASK                         (0x00400000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED13_SHIFT                        (0x00000016U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED13_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED13_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED14_MASK                         (0x00800000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED14_SHIFT                        (0x00000017U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED14_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED14_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED15_MASK                         (0x01000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED15_SHIFT                        (0x00000018U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED15_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED15_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED16_MASK                         (0x02000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED16_SHIFT                        (0x00000019U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED16_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED16_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_IFA_I_LDO_STATUS_MASK               (0x04000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_IFA_I_LDO_STATUS_SHIFT              (0x0000001AU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_IFA_I_LDO_STATUS_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_IFA_I_LDO_STATUS_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED17_MASK                         (0x08000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED17_SHIFT                        (0x0000001BU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED17_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED17_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_ANA_LDO_STATUS_MASK           (0x10000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_ANA_LDO_STATUS_SHIFT          (0x0000001CU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_ANA_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_ANA_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_DIG_LDO_STATUS_MASK           (0x20000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_DIG_LDO_STATUS_SHIFT          (0x0000001DU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_DIG_LDO_STATUS_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RX4_ADC_I_DIG_LDO_STATUS_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED18_MASK                         (0x40000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED18_SHIFT                        (0x0000001EU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED18_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED18_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED19_MASK                         (0x80000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED19_SHIFT                        (0x0000001FU)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED19_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESERVED19_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG1_RESETVAL                                (0x00000000U)

/* RX_STATUS_REG2 */

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX1_ADC_I_AUTO_RST_MASK                 (0x00000001U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX1_ADC_I_AUTO_RST_SHIFT                (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX1_ADC_I_AUTO_RST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX1_ADC_I_AUTO_RST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED0_MASK                          (0x000000FEU)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED0_SHIFT                         (0x00000001U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED0_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED0_MAX                           (0x0000007FU)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX2_ADC_I_AUTO_RST_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX2_ADC_I_AUTO_RST_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX2_ADC_I_AUTO_RST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX2_ADC_I_AUTO_RST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED1_MASK                          (0x0000FE00U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED1_SHIFT                         (0x00000009U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED1_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED1_MAX                           (0x0000007FU)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX3_ADC_I_AUTO_RST_MASK                 (0x00010000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX3_ADC_I_AUTO_RST_SHIFT                (0x00000010U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX3_ADC_I_AUTO_RST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX3_ADC_I_AUTO_RST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED2_MASK                          (0x00FE0000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED2_SHIFT                         (0x00000011U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED2_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED2_MAX                           (0x0000007FU)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX4_ADC_I_AUTO_RST_MASK                 (0x01000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX4_ADC_I_AUTO_RST_SHIFT                (0x00000018U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX4_ADC_I_AUTO_RST_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RX4_ADC_I_AUTO_RST_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED3_MASK                          (0xFE000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED3_SHIFT                         (0x00000019U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED3_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESERVED3_MAX                           (0x0000007FU)

#define CSL_AR_RFANACIO_RX_STATUS_REG2_RESETVAL                                (0x00000000U)

/* LODIST_STATUS_REG */

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED6_MASK                       (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED6_SHIFT                      (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED6_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED6_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED0_MASK                       (0x0000001EU)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED0_SHIFT                      (0x00000001U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED0_MAX                        (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED7_MASK                       (0x00000020U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED7_SHIFT                      (0x00000005U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED7_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED7_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED1_MASK                       (0x000003C0U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED1_SHIFT                      (0x00000006U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED1_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED1_MAX                        (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED8_MASK                       (0x00000400U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED8_SHIFT                      (0x0000000AU)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED8_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED8_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED2_MASK                       (0x00007800U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED2_SHIFT                      (0x0000000BU)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED2_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED2_MAX                        (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED9_MASK                       (0x00008000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED9_SHIFT                      (0x0000000FU)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED9_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED9_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED3_MASK                       (0x000F0000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED3_SHIFT                      (0x00000010U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED3_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED3_MAX                        (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED10_MASK                      (0x00100000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED10_SHIFT                     (0x00000014U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED10_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED10_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED4_MASK                       (0x01E00000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED4_SHIFT                      (0x00000015U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED4_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED4_MAX                        (0x0000000FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_MULT_CHAIN_LDO_SC_OUT_MASK           (0x02000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_MULT_CHAIN_LDO_SC_OUT_SHIFT          (0x00000019U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_MULT_CHAIN_LDO_SC_OUT_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_MULT_CHAIN_LDO_SC_OUT_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED5_MASK                       (0xFC000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED5_SHIFT                      (0x0000001AU)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED5_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESERVED5_MAX                        (0x0000003FU)

#define CSL_AR_RFANACIO_LODIST_STATUS_REG_RESETVAL                             (0x00000000U)

/* TX_STATUS_REG */

#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO1_SC_OUT_MASK                     (0x00000001U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO1_SC_OUT_SHIFT                    (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO1_SC_OUT_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO1_SC_OUT_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO1_SC_OUT_MASK                      (0x00000002U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO1_SC_OUT_SHIFT                     (0x00000001U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO1_SC_OUT_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO1_SC_OUT_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO1_SC_OUT_MASK                  (0x00000004U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO1_SC_OUT_SHIFT                 (0x00000002U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO1_SC_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO1_SC_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO2_SC_OUT_MASK                     (0x00000008U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO2_SC_OUT_SHIFT                    (0x00000003U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO2_SC_OUT_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO2_SC_OUT_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO2_SC_OUT_MASK                      (0x00000010U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO2_SC_OUT_SHIFT                     (0x00000004U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO2_SC_OUT_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO2_SC_OUT_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO2_SC_OUT_MASK                  (0x00000020U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO2_SC_OUT_SHIFT                 (0x00000005U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO2_SC_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO2_SC_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO3_SC_OUT_MASK                     (0x00000040U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO3_SC_OUT_SHIFT                    (0x00000006U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO3_SC_OUT_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PPA_LDO3_SC_OUT_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO3_SC_OUT_MASK                      (0x00000080U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO3_SC_OUT_SHIFT                     (0x00000007U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO3_SC_OUT_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PS_LDO3_SC_OUT_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO3_SC_OUT_MASK                  (0x00000100U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO3_SC_OUT_SHIFT                 (0x00000008U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO3_SC_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_DAC_ANALDO3_SC_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_PA_LDO_SC_OUT_MASK                       (0x00000200U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PA_LDO_SC_OUT_SHIFT                      (0x00000009U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PA_LDO_SC_OUT_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_PA_LDO_SC_OUT_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_TX_STATUS_REG_RESERVED0_MASK                           (0xFFFFFC00U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_RESERVED0_SHIFT                          (0x0000000AU)
#define CSL_AR_RFANACIO_TX_STATUS_REG_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_TX_STATUS_REG_RESERVED0_MAX                            (0x003FFFFFU)

#define CSL_AR_RFANACIO_TX_STATUS_REG_RESETVAL                                 (0x00000000U)

/* WU_MODE_REG */

#define CSL_AR_RFANACIO_WU_MODE_REG_FUNC_TEST_DET_SYNC_MASK                    (0x00000001U)
#define CSL_AR_RFANACIO_WU_MODE_REG_FUNC_TEST_DET_SYNC_SHIFT                   (0x00000000U)
#define CSL_AR_RFANACIO_WU_MODE_REG_FUNC_TEST_DET_SYNC_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_WU_MODE_REG_FUNC_TEST_DET_SYNC_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_WU_MODE_REG_TEST_MODE_DET_SYNC_MASK                    (0x00000002U)
#define CSL_AR_RFANACIO_WU_MODE_REG_TEST_MODE_DET_SYNC_SHIFT                   (0x00000001U)
#define CSL_AR_RFANACIO_WU_MODE_REG_TEST_MODE_DET_SYNC_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_WU_MODE_REG_TEST_MODE_DET_SYNC_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_WU_MODE_REG_SOP_MODE_LAT_4_0_MASK                      (0x0000007CU)
#define CSL_AR_RFANACIO_WU_MODE_REG_SOP_MODE_LAT_4_0_SHIFT                     (0x00000002U)
#define CSL_AR_RFANACIO_WU_MODE_REG_SOP_MODE_LAT_4_0_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_MODE_REG_SOP_MODE_LAT_4_0_MAX                       (0x0000001FU)

#define CSL_AR_RFANACIO_WU_MODE_REG_RESERVED0_MASK                             (0xFFFFFF80U)
#define CSL_AR_RFANACIO_WU_MODE_REG_RESERVED0_SHIFT                            (0x00000007U)
#define CSL_AR_RFANACIO_WU_MODE_REG_RESERVED0_RESETVAL                         (0x00000000U)
#define CSL_AR_RFANACIO_WU_MODE_REG_RESERVED0_MAX                              (0x01FFFFFFU)

#define CSL_AR_RFANACIO_WU_MODE_REG_RESETVAL                                   (0x00000000U)

/* WU_STATUS_REG */

#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_OVDET_LAT_MASK                      (0x00000001U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_OVDET_LAT_SHIFT                     (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_OVDET_LAT_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_OVDET_LAT_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_UVDET_LAT_MASK                      (0x00000002U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_UVDET_LAT_SHIFT                     (0x00000001U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_UVDET_LAT_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_CORE_UVDET_LAT_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA18BB_UV_DET_LAT_MASK                 (0x00000004U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA18BB_UV_DET_LAT_SHIFT                (0x00000002U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA18BB_UV_DET_LAT_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA18BB_UV_DET_LAT_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_CLK18_MASK                       (0x00000008U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_CLK18_SHIFT                      (0x00000003U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_CLK18_RESETVAL                   (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_CLK18_MAX                        (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO18_MASK                        (0x00000010U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO18_SHIFT                       (0x00000004U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO18_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO18_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO33_MASK                        (0x00000020U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO33_SHIFT                       (0x00000005U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO33_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_IO33_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_RF10_MASK                        (0x00000040U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_RF10_SHIFT                       (0x00000006U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_RF10_RESETVAL                    (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_RF10_MAX                         (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF1_UVDET_LAT_MASK                 (0x00000080U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF1_UVDET_LAT_SHIFT                (0x00000007U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF1_UVDET_LAT_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF1_UVDET_LAT_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF2_UVDET_LAT_MASK                 (0x00000100U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF2_UVDET_LAT_SHIFT                (0x00000008U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF2_UVDET_LAT_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA10RF2_UVDET_LAT_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_SRAM12_MASK                      (0x00000200U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_SRAM12_SHIFT                     (0x00000009U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_SRAM12_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_SRAM12_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_VDDD18_MASK                      (0x00000400U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_VDDD18_SHIFT                     (0x0000000AU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_VDDD18_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_SUPP_OK_VDDD18_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_REF_CLK_STATUS_MASK                      (0x00000800U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_REF_CLK_STATUS_SHIFT                     (0x0000000BU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_REF_CLK_STATUS_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_REF_CLK_STATUS_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_RCOSC_CLK_STATUS_MASK                    (0x00001000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RCOSC_CLK_STATUS_SHIFT                   (0x0000000CU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RCOSC_CLK_STATUS_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RCOSC_CLK_STATUS_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_XTAL_DET_STATUS_MASK                     (0x00002000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_XTAL_DET_STATUS_SHIFT                    (0x0000000DU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_XTAL_DET_STATUS_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_XTAL_DET_STATUS_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_LIMP_MODE_STATUS_MASK                    (0x00004000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_LIMP_MODE_STATUS_SHIFT                   (0x0000000EU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_LIMP_MODE_STATUS_RESETVAL                (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_LIMP_MODE_STATUS_MAX                     (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_HVMODE_MASK                              (0x00008000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_HVMODE_SHIFT                             (0x0000000FU)
#define CSL_AR_RFANACIO_WU_STATUS_REG_HVMODE_RESETVAL                          (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_HVMODE_MAX                               (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_APLLVCO18_UVDET_LAT_MASK                 (0x00010000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_APLLVCO18_UVDET_LAT_SHIFT                (0x00000010U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_APLLVCO18_UVDET_LAT_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_APLLVCO18_UVDET_LAT_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA_OSC_UVDET_LAT_MASK                  (0x00020000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA_OSC_UVDET_LAT_SHIFT                 (0x00000011U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA_OSC_UVDET_LAT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDA_OSC_UVDET_LAT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDS_3P3V_UVDET_LAT_MASK                 (0x00040000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDS_3P3V_UVDET_LAT_SHIFT                (0x00000012U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDS_3P3V_UVDET_LAT_RESETVAL             (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_VDDS_3P3V_UVDET_LAT_MAX                  (0x00000001U)

#define CSL_AR_RFANACIO_WU_STATUS_REG_RESERVED0_MASK                           (0xFFF80000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RESERVED0_SHIFT                          (0x00000013U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RESERVED0_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_WU_STATUS_REG_RESERVED0_MAX                            (0x00001FFFU)

#define CSL_AR_RFANACIO_WU_STATUS_REG_RESETVAL                                 (0x00000000U)

/* WU_SPARE_OUT */

#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDCLK18DET_MASK                          (0x00000001U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDCLK18DET_SHIFT                         (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDCLK18DET_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDCLK18DET_MAX                           (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDARF_DET_MASK                           (0x00000002U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDARF_DET_SHIFT                          (0x00000001U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDARF_DET_RESETVAL                       (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDARF_DET_MAX                            (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDS18DET_MASK                            (0x00000004U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDS18DET_SHIFT                           (0x00000002U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDS18DET_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_VDDS18DET_MAX                             (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_HVMODE_MASK                               (0x00000008U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_HVMODE_SHIFT                              (0x00000003U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_HVMODE_RESETVAL                           (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_HVMODE_MAX                                (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_SUPPDET_OV_CTRL_MASK                      (0x00000010U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_SUPPDET_OV_CTRL_SHIFT                     (0x00000004U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_SUPPDET_OV_CTRL_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_SUPPDET_OV_CTRL_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_INT_OSC_CTRL_MASK                         (0x00000020U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_INT_OSC_CTRL_SHIFT                        (0x00000005U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_INT_OSC_CTRL_RESETVAL                     (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_INT_OSC_CTRL_MAX                          (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_OVDET_LOWV_MASK                      (0x00000040U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_OVDET_LOWV_SHIFT                     (0x00000006U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_OVDET_LOWV_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_OVDET_LOWV_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_UVDET_LOWV_MASK                      (0x00000080U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_UVDET_LOWV_SHIFT                     (0x00000007U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_UVDET_LOWV_RESETVAL                  (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_CORE_UVDET_LOWV_MAX                       (0x00000001U)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_RESERVED0_MASK                            (0xFFFFFF00U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_RESERVED0_SHIFT                           (0x00000008U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_RESERVED0_RESETVAL                        (0x00000000U)
#define CSL_AR_RFANACIO_WU_SPARE_OUT_RESERVED0_MAX                             (0x00FFFFFFU)

#define CSL_AR_RFANACIO_WU_SPARE_OUT_RESETVAL                                  (0x00000000U)

/* CLK_STATUS_REG */

#define CSL_AR_RFANACIO_CLK_STATUS_REG_SLICER_LDO_SC_OUT_MASK                  (0x00000001U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SLICER_LDO_SC_OUT_SHIFT                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SLICER_LDO_SC_OUT_RESETVAL              (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SLICER_LDO_SC_OUT_MAX                   (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_MASK                (0x00000002U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_SHIFT               (0x00000001U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_MASK       (0x00000004U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_SHIFT      (0x00000002U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_RESETVAL   (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_MAX        (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_SDM_LDO_SC_OUT_MASK                     (0x00000008U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SDM_LDO_SC_OUT_SHIFT                    (0x00000003U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SDM_LDO_SC_OUT_RESETVAL                 (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SDM_LDO_SC_OUT_MAX                      (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_MASK               (0x00000010U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_SHIFT              (0x00000004U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_MASK               (0x00000020U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_SHIFT              (0x00000005U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_RESETVAL           (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_MAX                (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_MASK      (0x00000040U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_SHIFT     (0x00000006U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_RESETVAL  (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_MAX       (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_MASK                (0x00000080U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_SHIFT               (0x00000007U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_RESETVAL            (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_MAX                 (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_MASK           (0x00000100U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_SHIFT          (0x00000008U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_RESETVAL       (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_MAX            (0x00000001U)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_RESERVED0_MASK                          (0xFFFFFE00U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_RESERVED0_SHIFT                         (0x00000009U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_RESERVED0_RESETVAL                      (0x00000000U)
#define CSL_AR_RFANACIO_CLK_STATUS_REG_RESERVED0_MAX                           (0x007FFFFFU)

#define CSL_AR_RFANACIO_CLK_STATUS_REG_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
