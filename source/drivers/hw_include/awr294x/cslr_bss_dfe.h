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
 *  Name        : cslr_bss_dfe.h
*/
#ifndef CSLR_BSS_DFE_H_
#define CSLR_BSS_DFE_H_

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
    volatile uint32_t S2P_DELAY_1;
    volatile uint32_t S2P_DELAY_2;
    volatile uint32_t DIG_INTF_CFG;
    volatile uint32_t DECIM_CHAIN_CFG;
    volatile uint32_t GD_EQ_COEFF_RX1_RX2;
    volatile uint32_t GD_EQ_COEFF_RX3_RX4;
    volatile uint32_t GD_EQ_BYPASS_CFG;
    volatile uint32_t IF_MM_HPF1_ICH_RX1;
    volatile uint32_t IF_MM_HPF2_ICH_RX1;
    volatile uint32_t IF_MM_LPF1_ICH_RX1;
    volatile uint32_t IF_MM_LPF2_ICH_RX1;
    volatile uint32_t IF_MM_ALPHA_BYP_ICH_RX1;
    volatile uint32_t IF_MM_HPF1_QCH_RX1;
    volatile uint32_t IF_MM_HPF2_QCH_RX1;
    volatile uint32_t IF_MM_LPF1_QCH_RX1;
    volatile uint32_t IF_MM_LPF2_QCH_RX1;
    volatile uint32_t IF_MM_ALPHA_BYP_QCH_RX1;
    volatile uint32_t IF_MM_HPF1_ICH_RX2;
    volatile uint32_t IF_MM_HPF2_ICH_RX2;
    volatile uint32_t IF_MM_LPF1_ICH_RX2;
    volatile uint32_t IF_MM_LPF2_ICH_RX2;
    volatile uint32_t IF_MM_ALPHA_BYP_ICH_RX2;
    volatile uint32_t IF_MM_HPF1_QCH_RX2;
    volatile uint32_t IF_MM_HPF2_QCH_RX2;
    volatile uint32_t IF_MM_LPF1_QCH_RX2;
    volatile uint32_t IF_MM_LPF2_QCH_RX2;
    volatile uint32_t IF_MM_ALPHA_BYP_QCH_RX2;
    volatile uint32_t IF_MM_HPF1_ICH_RX3;
    volatile uint32_t IF_MM_HPF2_ICH_RX3;
    volatile uint32_t IF_MM_LPF1_ICH_RX3;
    volatile uint32_t IF_MM_LPF2_ICH_RX3;
    volatile uint32_t IF_MM_ALPHA_BYP_ICH_RX3;
    volatile uint32_t IF_MM_HPF1_QCH_RX3;
    volatile uint32_t IF_MM_HPF2_QCH_RX3;
    volatile uint32_t IF_MM_LPF1_QCH_RX3;
    volatile uint32_t IF_MM_LPF2_QCH_RX3;
    volatile uint32_t IF_MM_ALPHA_BYP_QCH_RX3;
    volatile uint32_t IF_MM_HPF1_ICH_RX4;
    volatile uint32_t IF_MM_HPF2_ICH_RX4;
    volatile uint32_t IF_MM_LPF1_ICH_RX4;
    volatile uint32_t IF_MM_LPF2_ICH_RX4;
    volatile uint32_t IF_MM_ALPHA_BYP_ICH_RX4;
    volatile uint32_t IF_MM_HPF1_QCH_RX4;
    volatile uint32_t IF_MM_HPF2_QCH_RX4;
    volatile uint32_t IF_MM_LPF1_QCH_RX4;
    volatile uint32_t IF_MM_LPF2_QCH_RX4;
    volatile uint32_t IF_MM_ALPHA_BYP_QCH_RX4;
    volatile uint32_t IF_MM_LPF_HPF_BYP;
    volatile uint32_t IF_MM_ALPHA_BYPASS_CTRL;
    volatile uint32_t RESAMP_A8_COEFF;
    volatile uint32_t RESAMP_A8_COEFF_BYP_CFG;
    volatile uint32_t DC_CORR_RX1;
    volatile uint32_t DC_CORR_RX2;
    volatile uint32_t DC_CORR_RX3;
    volatile uint32_t DC_CORR_RX4;
    volatile uint32_t RX_IQMM_CORR_RX1;
    volatile uint32_t RX_IQMM_CORR_RX2;
    volatile uint32_t RX_IQMM_CORR_RX3;
    volatile uint32_t RX_IQMM_CORR_RX4;
    volatile uint32_t AN_UPN_FREQ_CFG_RX1;
    volatile uint32_t AN_UPN_FREQ_CFG_RX2;
    volatile uint32_t AN_UPN_FREQ_CFG_RX3;
    volatile uint32_t AN_UPN_FREQ_CFG_RX4;
    volatile uint32_t AN_UPN_FREQ_INIT_CFG_RX1;
    volatile uint32_t AN_UPN_FREQ_INIT_CFG_RX2;
    volatile uint32_t AN_UPN_FREQ_INIT_CFG_RX3;
    volatile uint32_t AN_UPN_FREQ_INIT_CFG_RX4;
    volatile uint32_t CENTERING_IMJREJ_FREQ_CFG_RX1;
    volatile uint32_t CENTERING_IMJREJ_FREQ_CFG_RX2;
    volatile uint32_t CENTERING_IMJREJ_FREQ_CFG_RX3;
    volatile uint32_t CENTERING_IMJREJ_FREQ_CFG_RX4;
    volatile uint32_t CENTERING_IMJREJ_FREQ_INIT_CFG_RX1;
    volatile uint32_t CENTERING_IMJREJ_FREQ_INIT_CFG_RX2;
    volatile uint32_t CENTERING_IMJREJ_FREQ_INIT_CFG_RX3;
    volatile uint32_t CENTERING_IMJREJ_FREQ_INIT_CFG_RX4;
    volatile uint32_t DECENT_FREQ_CFG_RX1;
    volatile uint32_t DECENT_FREQ_CFG_RX2;
    volatile uint32_t DECENT_FREQ_CFG_RX3;
    volatile uint32_t DECENT_FREQ_CFG_RX4;
    volatile uint32_t DECENT_FREQ_INIT_CFG_RX1;
    volatile uint32_t DECENT_FREQ_INIT_CFG_RX2;
    volatile uint32_t DECENT_FREQ_INIT_CFG_RX3;
    volatile uint32_t DECENT_FREQ_INIT_CFG_RX4;
    volatile uint32_t DFE_OUTPUT_MODE_SELECTOR;
    volatile uint32_t BUMPER_SIG_CFG;
    volatile uint32_t AGC_IQMM;
    volatile uint32_t WIDE_BAND_INTF_1;
    volatile uint32_t WIDE_BAND_INTF_2;
    volatile uint32_t WIDE_BAND_INTF_3;
    volatile uint32_t WIDE_BAND_INTF_4;
    volatile uint32_t WIDE_BAND_INTF_5;
    volatile uint32_t WIDE_BAND_INTF_6;
    volatile uint32_t WIDE_BAND_INTF_7;
    volatile uint32_t WIDE_BAND_INTF_8;
    volatile uint32_t WIDE_BAND_INTF_9;
    volatile uint32_t WIDE_BAND_INTF_10;
    volatile uint32_t WIDE_BAND_INTF_11;
    volatile uint32_t WIDE_BAND_INTF_12;
    volatile uint32_t WIDE_BAND_INTF_13;
    volatile uint32_t WIDE_BAND_INTF_14;
    volatile uint32_t WIDE_BAND_INTF_15;
    volatile uint32_t SIG_IMG_BAND_1;
    volatile uint32_t SIG_IMG_BAND_2;
    volatile uint32_t RF_RESPOSE_EQ_RX_CFG1;
    volatile uint32_t RF_RESPOSE_EQ_RX_CFG2;
    volatile uint32_t LFSR_RX_IFMM_ICH_RX1_CFG;
    volatile uint32_t LFSR_RX_IFMM_ICH_RX2_CFG;
    volatile uint32_t LFSR_RX_IFMM_ICH_RX3_CFG;
    volatile uint32_t LFSR_RX_IFMM_ICH_RX4_CFG;
    volatile uint32_t LFSR_RX_IFMM_QCH_RX1_CFG;
    volatile uint32_t LFSR_RX_IFMM_QCH_RX2_CFG;
    volatile uint32_t LFSR_RX_IFMM_QCH_RX3_CFG;
    volatile uint32_t LFSR_RX_IFMM_QCH_RX4_CFG;
    volatile uint32_t LFSR_RX_FREQ_SHIFT_1_CFG;
    volatile uint32_t LFSR_RX_FREQ_SHIFT_2_CFG;
    volatile uint32_t DECIM_PHASE_SELECT_CONFIG;
    volatile uint32_t LFSR_LOAD_CONFIG;
    volatile uint32_t ADC_VALID_CNTR_STAT_COLL0_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_STAT_COLL0_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_STAT_COLL1_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_STAT_COLL1_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_BUMP_SIG_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_BUMP_SIG_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_WIDE_BAND_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_WIDE_BAND_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_DFE_OUT_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_DFE_OUT_END_CFG;
    volatile uint32_t ADC_VALID_CNTR_DFE_IN_START_CFG;
    volatile uint32_t ADC_VALID_CNTR_DFE_IN_END_CFG;
    volatile uint32_t RAMP_GEN_CONFIG;
    volatile uint32_t DFE_CHAIN_ENABLES;
    volatile uint32_t DFE_PARITY_CFG;
    volatile uint32_t DFE_DEBUG_CFG1;
    volatile uint32_t DFE_DSPSS_INTF_CFG1;
    volatile uint32_t DFE_DSPSS_INTF_CFG2;
    volatile uint32_t IFMM_EQ_HPF_ERROR_SIG_SWAP;
    volatile uint32_t CQ_ADC_SATURATION_CFG;
    volatile uint32_t DFE_OUTPUT_BIT_SELECTION;
    volatile uint32_t CQ_SPI_MODE_CFG;
    volatile uint32_t AGC_ECC_CFG;
    volatile uint32_t FIR_3TAP_CFG;
    volatile uint32_t CQ_CHIRP_PROFILE_CFG1;
    volatile uint32_t CQ_CHIRP_PROFILE_CFG2;
    volatile uint32_t RF_RESPOSE_EQ_RX_CFG3;
    volatile uint8_t  Resv_592[8];
    volatile uint32_t TEST_SRC_CTRL;
    volatile uint32_t BEAT_FREQ_TS1;
    volatile uint32_t PHI_INIT_TS1;
    volatile uint32_t BEAT_FREQ_TS2;
    volatile uint32_t PHI_INIT_TS2;
    volatile uint32_t BEAT_FREQ_TS3;
    volatile uint32_t PHI_INIT_TS3;
    volatile uint32_t DF_PROFILE0_TS1;
    volatile uint32_t DF_PROFILE1_TS1;
    volatile uint32_t DF_PROFILE2_TS1;
    volatile uint32_t DF_PROFILE3_TS1;
    volatile uint32_t DF_PROFILE4_TS1;
    volatile uint32_t DF_PROFILE5_TS1;
    volatile uint32_t DF_PROFILE6_TS1;
    volatile uint32_t DF_PROFILE7_TS1;
    volatile uint32_t DF_PROFILE8_TS1;
    volatile uint32_t DF_PROFILE9_TS1;
    volatile uint32_t DF_PROFILE10_TS1;
    volatile uint32_t DF_PROFILE11_TS1;
    volatile uint32_t DF_PROFILE12_TS1;
    volatile uint32_t DF_PROFILE13_TS1;
    volatile uint32_t DF_PROFILE14_TS1;
    volatile uint32_t DF_PROFILE15_TS1;
    volatile uint32_t DF_PROFILE0_TS2;
    volatile uint32_t DF_PROFILE1_TS2;
    volatile uint32_t DF_PROFILE2_TS2;
    volatile uint32_t DF_PROFILE3_TS2;
    volatile uint32_t DF_PROFILE4_TS2;
    volatile uint32_t DF_PROFILE5_TS2;
    volatile uint32_t DF_PROFILE6_TS2;
    volatile uint32_t DF_PROFILE7_TS2;
    volatile uint32_t DF_PROFILE8_TS2;
    volatile uint32_t DF_PROFILE9_TS2;
    volatile uint32_t DF_PROFILE10_TS2;
    volatile uint32_t DF_PROFILE11_TS2;
    volatile uint32_t DF_PROFILE12_TS2;
    volatile uint32_t DF_PROFILE13_TS2;
    volatile uint32_t DF_PROFILE14_TS2;
    volatile uint32_t DF_PROFILE15_TS2;
    volatile uint32_t DF_PROFILE0_TS3;
    volatile uint32_t DF_PROFILE1_TS3;
    volatile uint32_t DF_PROFILE2_TS3;
    volatile uint32_t DF_PROFILE3_TS3;
    volatile uint32_t DF_PROFILE4_TS3;
    volatile uint32_t DF_PROFILE5_TS3;
    volatile uint32_t DF_PROFILE6_TS3;
    volatile uint32_t DF_PROFILE7_TS3;
    volatile uint32_t DF_PROFILE8_TS3;
    volatile uint32_t DF_PROFILE9_TS3;
    volatile uint32_t DF_PROFILE10_TS3;
    volatile uint32_t DF_PROFILE11_TS3;
    volatile uint32_t DF_PROFILE12_TS3;
    volatile uint32_t DF_PROFILE13_TS3;
    volatile uint32_t DF_PROFILE14_TS3;
    volatile uint32_t DF_PROFILE15_TS3;
    volatile uint32_t AMPLITUDE_SCALE_TS;
    volatile uint32_t AMPLITUDE_SCALE_TS3;
    volatile uint32_t DELTA_FREQ_WORD_TS1;
    volatile uint32_t DELTA_FREQ_WORD_TS2;
    volatile uint32_t DELTA_FREQ_WORD_TS3;
    volatile uint32_t FREQ_SEED_TS1;
    volatile uint32_t FREQ_SEED_TS2;
    volatile uint32_t FREQ_SEED_TS3;
    volatile uint32_t MAX_FREQ_THRESHOLD;
    volatile uint32_t MIN_FREQ_THRESHOLD;
    volatile uint32_t DOA_ROT_TS1_RX1;
    volatile uint32_t DOA_ROT_TS2_RX1;
    volatile uint32_t DOA_ROT_TS1_RX2;
    volatile uint32_t DOA_ROT_TS2_RX2;
    volatile uint32_t DOA_ROT_TS1_RX3;
    volatile uint32_t DOA_ROT_TS2_RX3;
    volatile uint32_t DOA_ROT_TS1_RX4;
    volatile uint32_t DOA_ROT_TS2_RX4;
    volatile uint8_t  Resv_896[12];
    volatile uint32_t TX_DFE_CTRL;
    volatile uint32_t TX_BYPASS_CTRL;
    volatile uint32_t TX_PHASE_SHIFT_BYPASS_VAL_RF1;
    volatile uint32_t TX_PHASE_SHIFT_BYPASS_VAL_RF2;
    volatile uint32_t TX_PHASE_SHIFT_BYPASS_VAL_RF3;
    volatile uint32_t TX_IQMM_MUX_SEL;
    volatile uint32_t TX_IQMM_GAIN_PH_CORR_RF1;
    volatile uint32_t TX_IQMM_GAIN_PH_CORR_RF2;
    volatile uint32_t TX_IQMM_GAIN_PH_CORR_RF3;
    volatile uint32_t TX_RF_MM_EQ_TAU;
    volatile uint32_t TX_RF_MM_EQ_TAU_INIT_VAL;
    volatile uint32_t TX_PPD_MUVAL;
    volatile uint32_t TX_PPD_MUACC1;
    volatile uint32_t TX_PPD_MUACC2;
    volatile uint32_t TX_PPD_MUACC3;
    volatile uint32_t MUX_TXD_RF_CFG;
    volatile uint32_t TX_DC_CORR_RF1;
    volatile uint32_t TX_DC_CORR_RF2;
    volatile uint32_t TX_DC_CORR_RF3;
    volatile uint8_t  Resv_1024[52];
    volatile uint32_t RX_IQMM_CFG1;
    volatile uint32_t RX_IQMM_CFG2;
    volatile uint32_t RX_IQMM_CFG3;
    volatile uint32_t RX_IQMM_CFG4;
    volatile uint32_t CHIRP_MASK_CFG;
    volatile uint32_t CHIRP_COUNT_OVERRIDE_CFG;
    volatile uint32_t DFE_RESERVED_2;
    volatile uint32_t AGC_RX1_ICH_ACC_LSB;
    volatile uint32_t AGC_RX1_ICH_ACC_MSB;
    volatile uint32_t AGC_RX1_QCH_ACC_LSB;
    volatile uint32_t AGC_RX1_QCH_ACC_MSB;
    volatile uint32_t AGC_RX1_ISQ_ACC_LSB;
    volatile uint32_t AGC_RX1_ISQ_ACC_MSB;
    volatile uint32_t AGC_RX1_QSQ_ACC_LSB;
    volatile uint32_t AGC_RX1_QSQ_ACC_MSB;
    volatile uint32_t AGC_RX1_IQ_ACC_LSB;
    volatile uint32_t AGC_RX1_IQ_ACC_MSB;
    volatile uint32_t AGC_RX2_ICH_ACC_LSB;
    volatile uint32_t AGC_RX2_ICH_ACC_MSB;
    volatile uint32_t AGC_RX2_QCH_ACC_LSB;
    volatile uint32_t AGC_RX2_QCH_ACC_MSB;
    volatile uint32_t AGC_RX2_ISQ_ACC_LSB;
    volatile uint32_t AGC_RX2_ISQ_ACC_MSB;
    volatile uint32_t AGC_RX2_QSQ_ACC_LSB;
    volatile uint32_t AGC_RX2_QSQ_ACC_MSB;
    volatile uint32_t AGC_RX2_IQ_ACC_LSB;
    volatile uint32_t AGC_RX2_IQ_ACC_MSB;
    volatile uint32_t AGC_RX3_ICH_ACC_LSB;
    volatile uint32_t AGC_RX3_ICH_ACC_MSB;
    volatile uint8_t  Resv_1152[12];
    volatile uint32_t AGC_RX3_QCH_ACC_LSB;
    volatile uint32_t AGC_RX3_QCH_ACC_MSB;
    volatile uint32_t AGC_RX3_ISQ_ACC_LSB;
    volatile uint32_t AGC_RX3_ISQ_ACC_MSB;
    volatile uint32_t AGC_RX3_QSQ_ACC_LSB;
    volatile uint32_t AGC_RX3_QSQ_ACC_MSB;
    volatile uint32_t AGC_RX3_IQ_ACC_LSB;
    volatile uint32_t AGC_RX3_IQ_ACC_MSB;
    volatile uint32_t AGC_RX4_ICH_ACC_LSB;
    volatile uint32_t AGC_RX4_ICH_ACC_MSB;
    volatile uint32_t AGC_RX4_QCH_ACC_LSB;
    volatile uint32_t AGC_RX4_QCH_ACC_MSB;
    volatile uint32_t AGC_RX4_ISQ_ACC_LSB;
    volatile uint32_t AGC_RX4_ISQ_ACC_MSB;
    volatile uint32_t AGC_RX4_QSQ_ACC_LSB;
    volatile uint32_t AGC_RX4_QSQ_ACC_MSB;
    volatile uint32_t AGC_RX4_IQ_ACC_LSB;
    volatile uint32_t AGC_RX4_IQ_ACC_MSB;
    volatile uint32_t AGC_COUNT_ACC;
    volatile uint32_t PARITY_OR_STATUS;
    volatile uint32_t PARITY_AND_STATUS;
    volatile uint32_t TX_DFE_SWAP_FLIP;
    volatile uint32_t CQ3_RX1_IQ_RD;
    volatile uint32_t CQ3_RX2_IQ_RD;
    volatile uint32_t CQ3_RX3_IQ_RD;
    volatile uint32_t CQ3_RX4_IQ_RD;
    volatile uint32_t MUX_TXD_RF_CFG_TX4;
    volatile uint32_t TX4_DFE_CFG;
    volatile uint32_t TX_PHASE_SHIFT_BYPASS_VAL_RF4;
} CSL_bss_dfeRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_DFE_S2P_DELAY_1                                                (0x00000000U)
#define CSL_BSS_DFE_S2P_DELAY_2                                                (0x00000004U)
#define CSL_BSS_DFE_DIG_INTF_CFG                                               (0x00000008U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG                                            (0x0000000CU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2                                        (0x00000010U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4                                        (0x00000014U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG                                           (0x00000018U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1                                         (0x0000001CU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1                                         (0x00000020U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1                                         (0x00000024U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1                                         (0x00000028U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1                                    (0x0000002CU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1                                         (0x00000030U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1                                         (0x00000034U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1                                         (0x00000038U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1                                         (0x0000003CU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1                                    (0x00000040U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2                                         (0x00000044U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2                                         (0x00000048U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2                                         (0x0000004CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2                                         (0x00000050U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2                                    (0x00000054U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2                                         (0x00000058U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2                                         (0x0000005CU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2                                         (0x00000060U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2                                         (0x00000064U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2                                    (0x00000068U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3                                         (0x0000006CU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3                                         (0x00000070U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3                                         (0x00000074U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3                                         (0x00000078U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3                                    (0x0000007CU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3                                         (0x00000080U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3                                         (0x00000084U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3                                         (0x00000088U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3                                         (0x0000008CU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3                                    (0x00000090U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4                                         (0x00000094U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4                                         (0x00000098U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4                                         (0x0000009CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4                                         (0x000000A0U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4                                    (0x000000A4U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4                                         (0x000000A8U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4                                         (0x000000ACU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4                                         (0x000000B0U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4                                         (0x000000B4U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4                                    (0x000000B8U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP                                          (0x000000BCU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL                                    (0x000000C0U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF                                            (0x000000C4U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG                                    (0x000000C8U)
#define CSL_BSS_DFE_DC_CORR_RX1                                                (0x000000CCU)
#define CSL_BSS_DFE_DC_CORR_RX2                                                (0x000000D0U)
#define CSL_BSS_DFE_DC_CORR_RX3                                                (0x000000D4U)
#define CSL_BSS_DFE_DC_CORR_RX4                                                (0x000000D8U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1                                           (0x000000DCU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2                                           (0x000000E0U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3                                           (0x000000E4U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4                                           (0x000000E8U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1                                        (0x000000ECU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2                                        (0x000000F0U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3                                        (0x000000F4U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4                                        (0x000000F8U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1                                   (0x000000FCU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2                                   (0x00000100U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3                                   (0x00000104U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4                                   (0x00000108U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1                              (0x0000010CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2                              (0x00000110U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3                              (0x00000114U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4                              (0x00000118U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1                         (0x0000011CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2                         (0x00000120U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3                         (0x00000124U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4                         (0x00000128U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1                                        (0x0000012CU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2                                        (0x00000130U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3                                        (0x00000134U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4                                        (0x00000138U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1                                   (0x0000013CU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2                                   (0x00000140U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3                                   (0x00000144U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4                                   (0x00000148U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR                                   (0x0000014CU)
#define CSL_BSS_DFE_BUMPER_SIG_CFG                                             (0x00000150U)
#define CSL_BSS_DFE_AGC_IQMM                                                   (0x00000154U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1                                           (0x00000158U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_2                                           (0x0000015CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_3                                           (0x00000160U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_4                                           (0x00000164U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_5                                           (0x00000168U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6                                           (0x0000016CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7                                           (0x00000170U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8                                           (0x00000174U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9                                           (0x00000178U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10                                          (0x0000017CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11                                          (0x00000180U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12                                          (0x00000184U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13                                          (0x00000188U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14                                          (0x0000018CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15                                          (0x00000190U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1                                             (0x00000194U)
#define CSL_BSS_DFE_SIG_IMG_BAND_2                                             (0x00000198U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1                                      (0x0000019CU)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2                                      (0x000001A0U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG                                   (0x000001A4U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG                                   (0x000001A8U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG                                   (0x000001ACU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG                                   (0x000001B0U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG                                   (0x000001B4U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG                                   (0x000001B8U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG                                   (0x000001BCU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG                                   (0x000001C0U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG                                   (0x000001C4U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG                                   (0x000001C8U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG                                  (0x000001CCU)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG                                           (0x000001D0U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG                        (0x000001D4U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG                          (0x000001D8U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG                        (0x000001DCU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG                          (0x000001E0U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG                          (0x000001E4U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG                            (0x000001E8U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG                         (0x000001ECU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG                           (0x000001F0U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG                      (0x000001F4U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG                        (0x000001F8U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG                           (0x000001FCU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG                             (0x00000200U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG                            (0x00000204U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG                              (0x00000208U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG                                            (0x0000020CU)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES                                          (0x00000210U)
#define CSL_BSS_DFE_DFE_PARITY_CFG                                             (0x00000214U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1                                             (0x00000218U)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1                                        (0x0000021CU)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2                                        (0x00000220U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP                                 (0x00000224U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG                                      (0x00000228U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION                                   (0x0000022CU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG                                            (0x00000230U)
#define CSL_BSS_DFE_AGC_ECC_CFG                                                (0x00000234U)
#define CSL_BSS_DFE_FIR_3TAP_CFG                                               (0x00000238U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1                                      (0x0000023CU)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2                                      (0x00000240U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3                                      (0x00000244U)
#define CSL_BSS_DFE_TEST_SRC_CTRL                                              (0x00000250U)
#define CSL_BSS_DFE_BEAT_FREQ_TS1                                              (0x00000254U)
#define CSL_BSS_DFE_PHI_INIT_TS1                                               (0x00000258U)
#define CSL_BSS_DFE_BEAT_FREQ_TS2                                              (0x0000025CU)
#define CSL_BSS_DFE_PHI_INIT_TS2                                               (0x00000260U)
#define CSL_BSS_DFE_BEAT_FREQ_TS3                                              (0x00000264U)
#define CSL_BSS_DFE_PHI_INIT_TS3                                               (0x00000268U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1                                            (0x0000026CU)
#define CSL_BSS_DFE_DF_PROFILE1_TS1                                            (0x00000270U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1                                            (0x00000274U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1                                            (0x00000278U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1                                            (0x0000027CU)
#define CSL_BSS_DFE_DF_PROFILE5_TS1                                            (0x00000280U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1                                            (0x00000284U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1                                            (0x00000288U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1                                            (0x0000028CU)
#define CSL_BSS_DFE_DF_PROFILE9_TS1                                            (0x00000290U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1                                           (0x00000294U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1                                           (0x00000298U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1                                           (0x0000029CU)
#define CSL_BSS_DFE_DF_PROFILE13_TS1                                           (0x000002A0U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1                                           (0x000002A4U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1                                           (0x000002A8U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2                                            (0x000002ACU)
#define CSL_BSS_DFE_DF_PROFILE1_TS2                                            (0x000002B0U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2                                            (0x000002B4U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2                                            (0x000002B8U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2                                            (0x000002BCU)
#define CSL_BSS_DFE_DF_PROFILE5_TS2                                            (0x000002C0U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2                                            (0x000002C4U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2                                            (0x000002C8U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2                                            (0x000002CCU)
#define CSL_BSS_DFE_DF_PROFILE9_TS2                                            (0x000002D0U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2                                           (0x000002D4U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2                                           (0x000002D8U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2                                           (0x000002DCU)
#define CSL_BSS_DFE_DF_PROFILE13_TS2                                           (0x000002E0U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2                                           (0x000002E4U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2                                           (0x000002E8U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3                                            (0x000002ECU)
#define CSL_BSS_DFE_DF_PROFILE1_TS3                                            (0x000002F0U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3                                            (0x000002F4U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3                                            (0x000002F8U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3                                            (0x000002FCU)
#define CSL_BSS_DFE_DF_PROFILE5_TS3                                            (0x00000300U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3                                            (0x00000304U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3                                            (0x00000308U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3                                            (0x0000030CU)
#define CSL_BSS_DFE_DF_PROFILE9_TS3                                            (0x00000310U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3                                           (0x00000314U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3                                           (0x00000318U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3                                           (0x0000031CU)
#define CSL_BSS_DFE_DF_PROFILE13_TS3                                           (0x00000320U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3                                           (0x00000324U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3                                           (0x00000328U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS                                         (0x0000032CU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3                                        (0x00000330U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1                                        (0x00000334U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2                                        (0x00000338U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3                                        (0x0000033CU)
#define CSL_BSS_DFE_FREQ_SEED_TS1                                              (0x00000340U)
#define CSL_BSS_DFE_FREQ_SEED_TS2                                              (0x00000344U)
#define CSL_BSS_DFE_FREQ_SEED_TS3                                              (0x00000348U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD                                         (0x0000034CU)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD                                         (0x00000350U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1                                            (0x00000354U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1                                            (0x00000358U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2                                            (0x0000035CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2                                            (0x00000360U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3                                            (0x00000364U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3                                            (0x00000368U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4                                            (0x0000036CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4                                            (0x00000370U)
#define CSL_BSS_DFE_TX_DFE_CTRL                                                (0x00000380U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL                                             (0x00000384U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1                              (0x00000388U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2                              (0x0000038CU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3                              (0x00000390U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL                                            (0x00000394U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1                                   (0x00000398U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2                                   (0x0000039CU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3                                   (0x000003A0U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU                                            (0x000003A4U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL                                   (0x000003A8U)
#define CSL_BSS_DFE_TX_PPD_MUVAL                                               (0x000003ACU)
#define CSL_BSS_DFE_TX_PPD_MUACC1                                              (0x000003B0U)
#define CSL_BSS_DFE_TX_PPD_MUACC2                                              (0x000003B4U)
#define CSL_BSS_DFE_TX_PPD_MUACC3                                              (0x000003B8U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG                                             (0x000003BCU)
#define CSL_BSS_DFE_TX_DC_CORR_RF1                                             (0x000003C0U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2                                             (0x000003C4U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3                                             (0x000003C8U)
#define CSL_BSS_DFE_RX_IQMM_CFG1                                               (0x00000400U)
#define CSL_BSS_DFE_RX_IQMM_CFG2                                               (0x00000404U)
#define CSL_BSS_DFE_RX_IQMM_CFG3                                               (0x00000408U)
#define CSL_BSS_DFE_RX_IQMM_CFG4                                               (0x0000040CU)
#define CSL_BSS_DFE_CHIRP_MASK_CFG                                             (0x00000410U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG                                   (0x00000414U)
#define CSL_BSS_DFE_DFE_RESERVED_2                                             (0x00000418U)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB                                        (0x0000041CU)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB                                        (0x00000420U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB                                        (0x00000424U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB                                        (0x00000428U)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB                                        (0x0000042CU)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB                                        (0x00000430U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB                                        (0x00000434U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB                                        (0x00000438U)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB                                         (0x0000043CU)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB                                         (0x00000440U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB                                        (0x00000444U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB                                        (0x00000448U)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB                                        (0x0000044CU)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB                                        (0x00000450U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB                                        (0x00000454U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB                                        (0x00000458U)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB                                        (0x0000045CU)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB                                        (0x00000460U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB                                         (0x00000464U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB                                         (0x00000468U)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB                                        (0x0000046CU)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB                                        (0x00000470U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB                                        (0x00000480U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB                                        (0x00000484U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB                                        (0x00000488U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB                                        (0x0000048CU)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB                                        (0x00000490U)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB                                        (0x00000494U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB                                         (0x00000498U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB                                         (0x0000049CU)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB                                        (0x000004A0U)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB                                        (0x000004A4U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB                                        (0x000004A8U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB                                        (0x000004ACU)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB                                        (0x000004B0U)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB                                        (0x000004B4U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB                                        (0x000004B8U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB                                        (0x000004BCU)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB                                         (0x000004C0U)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB                                         (0x000004C4U)
#define CSL_BSS_DFE_AGC_COUNT_ACC                                              (0x000004C8U)
#define CSL_BSS_DFE_PARITY_OR_STATUS                                           (0x000004CCU)
#define CSL_BSS_DFE_PARITY_AND_STATUS                                          (0x000004D0U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP                                           (0x000004D4U)
#define CSL_BSS_DFE_CQ3_RX1_IQ_RD                                              (0x000004D8U)
#define CSL_BSS_DFE_CQ3_RX2_IQ_RD                                              (0x000004DCU)
#define CSL_BSS_DFE_CQ3_RX3_IQ_RD                                              (0x000004E0U)
#define CSL_BSS_DFE_CQ3_RX4_IQ_RD                                              (0x000004E4U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4                                         (0x000004E8U)
#define CSL_BSS_DFE_TX4_DFE_CFG                                                (0x000004ECU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4                              (0x000004F0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* S2P_DELAY_1 */

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX1_MASK               (0x0000000FU)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX1_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX1_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX1_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX1_MASK               (0x000000F0U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX1_SHIFT              (0x00000004U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX1_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX1_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX2_MASK               (0x00000F00U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX2_SHIFT              (0x00000008U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX2_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX2_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX2_MASK               (0x0000F000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX2_SHIFT              (0x0000000CU)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX2_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX2_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX3_MASK               (0x000F0000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX3_SHIFT              (0x00000010U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX3_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX3_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX3_MASK               (0x00F00000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX3_SHIFT              (0x00000014U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX3_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX3_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX4_MASK               (0x0F000000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX4_SHIFT              (0x00000018U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX4_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_ICH_RX4_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX4_MASK               (0xF0000000U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX4_SHIFT              (0x0000001CU)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX4_RESETVAL           (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_1_ADC_S2P_DELAY_NSAMP_QCH_RX4_MAX                (0x0000000FU)

#define CSL_BSS_DFE_S2P_DELAY_1_RESETVAL                                       (0x66666666U)

/* S2P_DELAY_2 */

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX1_MASK           (0x00000003U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX1_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX1_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX1_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX1_MASK           (0x0000000CU)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX1_SHIFT          (0x00000002U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX1_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX1_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX2_MASK           (0x00000030U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX2_SHIFT          (0x00000004U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX2_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX2_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX2_MASK           (0x000000C0U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX2_SHIFT          (0x00000006U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX2_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX2_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX3_MASK           (0x00000300U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX3_SHIFT          (0x00000008U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX3_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX3_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX3_MASK           (0x00000C00U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX3_SHIFT          (0x0000000AU)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX3_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX3_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX4_MASK           (0x00003000U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX4_SHIFT          (0x0000000CU)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX4_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_ICH_RX4_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX4_MASK           (0x0000C000U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX4_SHIFT          (0x0000000EU)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX4_RESETVAL       (0x00000001U)
#define CSL_BSS_DFE_S2P_DELAY_2_SINC_OUTPUT_DELAY_NSAMP_QCH_RX4_MAX            (0x00000003U)

#define CSL_BSS_DFE_S2P_DELAY_2_NU_MASK                                        (0xFFFF0000U)
#define CSL_BSS_DFE_S2P_DELAY_2_NU_SHIFT                                       (0x00000010U)
#define CSL_BSS_DFE_S2P_DELAY_2_NU_RESETVAL                                    (0x00000000U)
#define CSL_BSS_DFE_S2P_DELAY_2_NU_MAX                                         (0x0000FFFFU)

#define CSL_BSS_DFE_S2P_DELAY_2_RESETVAL                                       (0x00005555U)

/* DIG_INTF_CFG */

#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX1_MASK                              (0x00000001U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX1_SHIFT                             (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX1_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX1_MAX                               (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX2_MASK                              (0x00000002U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX2_SHIFT                             (0x00000001U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX2_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX2_MAX                               (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX3_MASK                              (0x00000004U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX3_SHIFT                             (0x00000002U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX3_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX3_MAX                               (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX4_MASK                              (0x00000008U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX4_SHIFT                             (0x00000003U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX4_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_IQ_SWAP_RX4_MAX                               (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX1_MASK                             (0x00000010U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX1_SHIFT                            (0x00000004U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX1_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX2_MASK                             (0x00000020U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX2_SHIFT                            (0x00000005U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX2_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX3_MASK                             (0x00000040U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX3_SHIFT                            (0x00000006U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX3_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX4_MASK                             (0x00000080U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX4_SHIFT                            (0x00000007U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX4_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_I_RX4_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX1_MASK                             (0x00000100U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX1_SHIFT                            (0x00000008U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX1_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX2_MASK                             (0x00000200U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX2_SHIFT                            (0x00000009U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX2_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX3_MASK                             (0x00000400U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX3_SHIFT                            (0x0000000AU)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX3_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX4_MASK                             (0x00000800U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX4_SHIFT                            (0x0000000BU)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX4_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NEGATE_Q_RX4_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX1_MASK                           (0x00001000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX1_SHIFT                          (0x0000000CU)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX1_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX2_MASK                           (0x00002000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX2_SHIFT                          (0x0000000DU)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX2_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX3_MASK                           (0x00004000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX3_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX4_MASK                           (0x00008000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX4_SHIFT                          (0x0000000FU)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_I_RX4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX1_MASK                           (0x00010000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX1_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX1_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX2_MASK                           (0x00020000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX2_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX2_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX3_MASK                           (0x00040000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX3_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX4_MASK                           (0x00080000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX4_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ZERO_OUT_Q_RX4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_ADC_DATA_MSB_LSB_FLIP_MASK                    (0x00100000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ADC_DATA_MSB_LSB_FLIP_SHIFT                   (0x00000014U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ADC_DATA_MSB_LSB_FLIP_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_ADC_DATA_MSB_LSB_FLIP_MAX                     (0x00000001U)

#define CSL_BSS_DFE_DIG_INTF_CFG_NU_MASK                                       (0xFFE00000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NU_SHIFT                                      (0x00000015U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_DIG_INTF_CFG_NU_MAX                                        (0x000007FFU)

#define CSL_BSS_DFE_DIG_INTF_CFG_RESETVAL                                      (0x00000000U)

/* DECIM_CHAIN_CFG */

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU1_MASK                                   (0x00000001U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU1_SHIFT                                  (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU1_MAX                                    (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A2_ENABLE_MASK                             (0x00000002U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A2_ENABLE_SHIFT                            (0x00000001U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A2_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A2_ENABLE_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A3_ENABLE_MASK                             (0x00000004U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A3_ENABLE_SHIFT                            (0x00000002U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A3_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A3_ENABLE_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A4_ENABLE_MASK                             (0x00000008U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A4_ENABLE_SHIFT                            (0x00000003U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A4_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A4_ENABLE_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A5_ENABLE_MASK                             (0x00000010U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A5_ENABLE_SHIFT                            (0x00000004U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A5_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A5_ENABLE_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A6_ENABLE_MASK                             (0x00000020U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A6_ENABLE_SHIFT                            (0x00000005U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A6_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_A6_ENABLE_MAX                              (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_SEL_MASK                           (0x000000C0U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_SEL_SHIFT                          (0x00000006U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_SEL_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_SEL_MAX                            (0x00000003U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_IQ_SEL_MASK                        (0x00000100U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_IQ_SEL_SHIFT                       (0x00000008U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_IQ_SEL_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_DFE_CLK_IQ_SEL_MAX                         (0x00000001U)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU2_MASK                                   (0xFFFFFE00U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU2_SHIFT                                  (0x00000009U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DECIM_CHAIN_CFG_NU2_MAX                                    (0x007FFFFFU)

#define CSL_BSS_DFE_DECIM_CHAIN_CFG_RESETVAL                                   (0x00000000U)

/* GD_EQ_COEFF_RX1_RX2 */

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX1_MASK           (0x0000007FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX1_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX1_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU1_MASK                               (0x00000080U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU1_SHIFT                              (0x00000007U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU1_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX1_MASK           (0x00007F00U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX1_SHIFT          (0x00000008U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX1_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU2_MASK                               (0x00008000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU2_SHIFT                              (0x0000000FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU2_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX2_MASK           (0x007F0000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX2_SHIFT          (0x00000010U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_ICH_RX2_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU3_MASK                               (0x00800000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU3_SHIFT                              (0x00000017U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU3_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX2_MASK           (0x7F000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX2_SHIFT          (0x00000018U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_IF_GD_EQ_COEFFT_QCH_RX2_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU4_MASK                               (0x80000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU4_SHIFT                              (0x0000001FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU4_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_NU4_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX1_RX2_RESETVAL                               (0x00000000U)

/* GD_EQ_COEFF_RX3_RX4 */

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX3_MASK           (0x0000007FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX3_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX3_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU1_MASK                               (0x00000080U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU1_SHIFT                              (0x00000007U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU1_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX3_MASK           (0x00007F00U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX3_SHIFT          (0x00000008U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX3_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU2_MASK                               (0x00008000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU2_SHIFT                              (0x0000000FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU2_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX4_MASK           (0x007F0000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX4_SHIFT          (0x00000010U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_ICH_RX4_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU3_MASK                               (0x00800000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU3_SHIFT                              (0x00000017U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU3_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX4_MASK           (0x7F000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX4_SHIFT          (0x00000018U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_IF_GD_EQ_COEFFT_QCH_RX4_MAX            (0x0000007FU)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU4_MASK                               (0x80000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU4_SHIFT                              (0x0000001FU)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU4_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_NU4_MAX                                (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_COEFF_RX3_RX4_RESETVAL                               (0x00000000U)

/* GD_EQ_BYPASS_CFG */

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX1_MASK                 (0x00000001U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX1_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX1_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX1_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX1_MASK                 (0x00000002U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX1_SHIFT                (0x00000001U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX1_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX1_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX2_MASK                 (0x00000004U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX2_SHIFT                (0x00000002U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX2_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX2_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX2_MASK                 (0x00000008U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX2_SHIFT                (0x00000003U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX2_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX2_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX3_MASK                 (0x00000010U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX3_SHIFT                (0x00000004U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX3_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX3_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX3_MASK                 (0x00000020U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX3_SHIFT                (0x00000005U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX3_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX3_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX4_MASK                 (0x00000040U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX4_SHIFT                (0x00000006U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX4_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_ICH_RX4_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX4_MASK                 (0x00000080U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX4_SHIFT                (0x00000007U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX4_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_IF_GD_EQ_BYP_QCH_RX4_MAX                  (0x00000001U)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_NU_MASK                                   (0xFFFFFF00U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_NU_SHIFT                                  (0x00000008U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_NU_MAX                                    (0x00FFFFFFU)

#define CSL_BSS_DFE_GD_EQ_BYPASS_CFG_RESETVAL                                  (0x00000000U)

/* IF_MM_HPF1_ICH_RX1 */

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_R_ICH_RX1_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_R_ICH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_R_ICH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_R_ICH_RX1_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_DELTA_R_ICH_RX1_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_DELTA_R_ICH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_DELTA_R_ICH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_IF_MM_EQ_HPF1_DELTA_R_ICH_RX1_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_ICH_RX1 */

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_R_ICH_RX1_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_R_ICH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_R_ICH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_R_ICH_RX1_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU3_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU3_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU3_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU3_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_DELTA_R_ICH_RX1_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_DELTA_R_ICH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_DELTA_R_ICH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_IF_MM_EQ_HPF2_DELTA_R_ICH_RX1_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU4_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU4_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU4_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_NU4_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_ICH_RX1 */

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_R_ICH_RX1_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_R_ICH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_R_ICH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_R_ICH_RX1_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_DELTA_R_ICH_RX1_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_DELTA_R_ICH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_DELTA_R_ICH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_IF_MM_EQ_LPF1_DELTA_R_ICH_RX1_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_ICH_RX1 */

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_R_ICH_RX1_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_R_ICH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_R_ICH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_R_ICH_RX1_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU3_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU3_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU3_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU3_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_DELTA_R_ICH_RX1_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_DELTA_R_ICH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_DELTA_R_ICH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_IF_MM_EQ_LPF2_DELTA_R_ICH_RX1_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU4_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU4_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU4_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_NU4_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_ICH_RX1 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_IF_MM_EQ_GAIN_ALPHA_ICH_RX1_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_IF_MM_EQ_GAIN_ALPHA_ICH_RX1_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_IF_MM_EQ_GAIN_ALPHA_ICH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_IF_MM_EQ_GAIN_ALPHA_ICH_RX1_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX1_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_QCH_RX1 */

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_R_QCH_RX1_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_R_QCH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_R_QCH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_R_QCH_RX1_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_DELTA_R_QCH_RX1_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_DELTA_R_QCH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_DELTA_R_QCH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_IF_MM_EQ_HPF1_DELTA_R_QCH_RX1_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_QCH_RX1 */

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_R_QCH_RX1_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_R_QCH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_R_QCH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_R_QCH_RX1_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_DELTA_R_QCH_RX1_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_DELTA_R_QCH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_DELTA_R_QCH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_IF_MM_EQ_HPF2_DELTA_R_QCH_RX1_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_QCH_RX1 */

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_R_QCH_RX1_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_R_QCH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_R_QCH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_R_QCH_RX1_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_DELTA_R_QCH_RX1_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_DELTA_R_QCH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_DELTA_R_QCH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_IF_MM_EQ_LPF1_DELTA_R_QCH_RX1_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_QCH_RX1 */

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_R_QCH_RX1_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_R_QCH_RX1_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_R_QCH_RX1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_R_QCH_RX1_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_DELTA_R_QCH_RX1_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_DELTA_R_QCH_RX1_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_DELTA_R_QCH_RX1_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_IF_MM_EQ_LPF2_DELTA_R_QCH_RX1_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX1_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_QCH_RX1 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_IF_MM_EQ_GAIN_ALPHA_QCH_RX1_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_IF_MM_EQ_GAIN_ALPHA_QCH_RX1_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_IF_MM_EQ_GAIN_ALPHA_QCH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_IF_MM_EQ_GAIN_ALPHA_QCH_RX1_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX1_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_ICH_RX2 */

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_R_ICH_RX2_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_R_ICH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_R_ICH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_R_ICH_RX2_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_DELTA_R_ICH_RX2_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_DELTA_R_ICH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_DELTA_R_ICH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_IF_MM_EQ_HPF1_DELTA_R_ICH_RX2_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_ICH_RX2 */

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_R_ICH_RX2_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_R_ICH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_R_ICH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_R_ICH_RX2_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_DELTA_R_ICH_RX2_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_DELTA_R_ICH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_DELTA_R_ICH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_IF_MM_EQ_HPF2_DELTA_R_ICH_RX2_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_ICH_RX2 */

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_R_ICH_RX2_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_R_ICH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_R_ICH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_R_ICH_RX2_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_DELTA_R_ICH_RX2_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_DELTA_R_ICH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_DELTA_R_ICH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_IF_MM_EQ_LPF1_DELTA_R_ICH_RX2_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_ICH_RX2 */

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_R_ICH_RX2_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_R_ICH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_R_ICH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_R_ICH_RX2_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_DELTA_R_ICH_RX2_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_DELTA_R_ICH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_DELTA_R_ICH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_IF_MM_EQ_LPF2_DELTA_R_ICH_RX2_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_ICH_RX2 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_IF_MM_EQ_GAIN_ALPHA_ICH_RX2_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_IF_MM_EQ_GAIN_ALPHA_ICH_RX2_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_IF_MM_EQ_GAIN_ALPHA_ICH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_IF_MM_EQ_GAIN_ALPHA_ICH_RX2_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX2_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_QCH_RX2 */

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_R_QCH_RX2_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_R_QCH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_R_QCH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_R_QCH_RX2_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_DELTA_R_QCH_RX2_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_DELTA_R_QCH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_DELTA_R_QCH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_IF_MM_EQ_HPF1_DELTA_R_QCH_RX2_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_QCH_RX2 */

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_R_QCH_RX2_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_R_QCH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_R_QCH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_R_QCH_RX2_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_DELTA_R_QCH_RX2_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_DELTA_R_QCH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_DELTA_R_QCH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_IF_MM_EQ_HPF2_DELTA_R_QCH_RX2_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_QCH_RX2 */

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_R_QCH_RX2_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_R_QCH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_R_QCH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_R_QCH_RX2_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_DELTA_R_QCH_RX2_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_DELTA_R_QCH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_DELTA_R_QCH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_IF_MM_EQ_LPF1_DELTA_R_QCH_RX2_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_QCH_RX2 */

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_R_QCH_RX2_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_R_QCH_RX2_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_R_QCH_RX2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_R_QCH_RX2_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_DELTA_R_QCH_RX2_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_DELTA_R_QCH_RX2_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_DELTA_R_QCH_RX2_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_IF_MM_EQ_LPF2_DELTA_R_QCH_RX2_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX2_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_QCH_RX2 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_IF_MM_EQ_GAIN_ALPHA_QCH_RX2_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_IF_MM_EQ_GAIN_ALPHA_QCH_RX2_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_IF_MM_EQ_GAIN_ALPHA_QCH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_IF_MM_EQ_GAIN_ALPHA_QCH_RX2_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU7_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU7_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU7_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_NU7_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX2_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_ICH_RX3 */

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_R_ICH_RX3_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_R_ICH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_R_ICH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_R_ICH_RX3_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_DELTA_R_ICH_RX3_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_DELTA_R_ICH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_DELTA_R_ICH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_IF_MM_EQ_HPF1_DELTA_R_ICH_RX3_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_ICH_RX3 */

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_R_ICH_RX3_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_R_ICH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_R_ICH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_R_ICH_RX3_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_DELTA_R_ICH_RX3_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_DELTA_R_ICH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_DELTA_R_ICH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_IF_MM_EQ_HPF2_DELTA_R_ICH_RX3_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_ICH_RX3 */

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_R_ICH_RX3_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_R_ICH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_R_ICH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_R_ICH_RX3_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_DELTA_R_ICH_RX3_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_DELTA_R_ICH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_DELTA_R_ICH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_IF_MM_EQ_LPF1_DELTA_R_ICH_RX3_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_ICH_RX3 */

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_R_ICH_RX3_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_R_ICH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_R_ICH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_R_ICH_RX3_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_DELTA_R_ICH_RX3_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_DELTA_R_ICH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_DELTA_R_ICH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_IF_MM_EQ_LPF2_DELTA_R_ICH_RX3_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_ICH_RX3 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_IF_MM_EQ_GAIN_ALPHA_ICH_RX3_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_IF_MM_EQ_GAIN_ALPHA_ICH_RX3_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_IF_MM_EQ_GAIN_ALPHA_ICH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_IF_MM_EQ_GAIN_ALPHA_ICH_RX3_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX3_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_QCH_RX3 */

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_R_QCH_RX3_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_R_QCH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_R_QCH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_R_QCH_RX3_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_DELTA_R_QCH_RX3_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_DELTA_R_QCH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_DELTA_R_QCH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_IF_MM_EQ_HPF1_DELTA_R_QCH_RX3_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_QCH_RX3 */

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_R_QCH_RX3_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_R_QCH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_R_QCH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_R_QCH_RX3_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_DELTA_R_QCH_RX3_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_DELTA_R_QCH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_DELTA_R_QCH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_IF_MM_EQ_HPF2_DELTA_R_QCH_RX3_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_QCH_RX3 */

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_R_QCH_RX3_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_R_QCH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_R_QCH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_R_QCH_RX3_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_DELTA_R_QCH_RX3_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_DELTA_R_QCH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_DELTA_R_QCH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_IF_MM_EQ_LPF1_DELTA_R_QCH_RX3_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_QCH_RX3 */

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_R_QCH_RX3_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_R_QCH_RX3_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_R_QCH_RX3_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_R_QCH_RX3_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_DELTA_R_QCH_RX3_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_DELTA_R_QCH_RX3_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_DELTA_R_QCH_RX3_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_IF_MM_EQ_LPF2_DELTA_R_QCH_RX3_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX3_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_QCH_RX3 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_IF_MM_EQ_GAIN_ALPHA_QCH_RX3_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_IF_MM_EQ_GAIN_ALPHA_QCH_RX3_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_IF_MM_EQ_GAIN_ALPHA_QCH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_IF_MM_EQ_GAIN_ALPHA_QCH_RX3_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX3_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_ICH_RX4 */

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_R_ICH_RX4_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_R_ICH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_R_ICH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_R_ICH_RX4_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_DELTA_R_ICH_RX4_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_DELTA_R_ICH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_DELTA_R_ICH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_IF_MM_EQ_HPF1_DELTA_R_ICH_RX4_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_ICH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_ICH_RX4 */

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_R_ICH_RX4_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_R_ICH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_R_ICH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_R_ICH_RX4_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_DELTA_R_ICH_RX4_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_DELTA_R_ICH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_DELTA_R_ICH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_IF_MM_EQ_HPF2_DELTA_R_ICH_RX4_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_ICH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_ICH_RX4 */

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_R_ICH_RX4_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_R_ICH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_R_ICH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_R_ICH_RX4_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_DELTA_R_ICH_RX4_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_DELTA_R_ICH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_DELTA_R_ICH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_IF_MM_EQ_LPF1_DELTA_R_ICH_RX4_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_ICH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_ICH_RX4 */

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_R_ICH_RX4_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_R_ICH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_R_ICH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_R_ICH_RX4_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_DELTA_R_ICH_RX4_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_DELTA_R_ICH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_DELTA_R_ICH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_IF_MM_EQ_LPF2_DELTA_R_ICH_RX4_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_ICH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_ICH_RX4 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_IF_MM_EQ_GAIN_ALPHA_ICH_RX4_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_IF_MM_EQ_GAIN_ALPHA_ICH_RX4_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_IF_MM_EQ_GAIN_ALPHA_ICH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_IF_MM_EQ_GAIN_ALPHA_ICH_RX4_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_ICH_RX4_RESETVAL                           (0x00000000U)

/* IF_MM_HPF1_QCH_RX4 */

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_R_QCH_RX4_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_R_QCH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_R_QCH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_R_QCH_RX4_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_DELTA_R_QCH_RX4_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_DELTA_R_QCH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_DELTA_R_QCH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_IF_MM_EQ_HPF1_DELTA_R_QCH_RX4_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF1_QCH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_HPF2_QCH_RX4 */

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_R_QCH_RX4_MASK            (0x00007FFFU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_R_QCH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_R_QCH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_R_QCH_RX4_MAX             (0x00007FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU1_MASK                                (0x00008000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU1_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU1_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_DELTA_R_QCH_RX4_MASK      (0x3FFF0000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_DELTA_R_QCH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_DELTA_R_QCH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_IF_MM_EQ_HPF2_DELTA_R_QCH_RX4_MAX       (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU2_MASK                                (0xC0000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU2_SHIFT                               (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_NU2_MAX                                 (0x00000003U)

#define CSL_BSS_DFE_IF_MM_HPF2_QCH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_LPF1_QCH_RX4 */

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_R_QCH_RX4_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_R_QCH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_R_QCH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_R_QCH_RX4_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_DELTA_R_QCH_RX4_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_DELTA_R_QCH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_DELTA_R_QCH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_IF_MM_EQ_LPF1_DELTA_R_QCH_RX4_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF1_QCH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_LPF2_QCH_RX4 */

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_R_QCH_RX4_MASK            (0x00000FFFU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_R_QCH_RX4_SHIFT           (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_R_QCH_RX4_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_R_QCH_RX4_MAX             (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU1_MASK                                (0x0000F000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU1_SHIFT                               (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_DELTA_R_QCH_RX4_MASK      (0x07FF0000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_DELTA_R_QCH_RX4_SHIFT     (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_DELTA_R_QCH_RX4_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_IF_MM_EQ_LPF2_DELTA_R_QCH_RX4_MAX       (0x000007FFU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU2_MASK                                (0xF8000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU2_SHIFT                               (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_NU2_MAX                                 (0x0000001FU)

#define CSL_BSS_DFE_IF_MM_LPF2_QCH_RX4_RESETVAL                                (0x00000000U)

/* IF_MM_ALPHA_BYP_QCH_RX4 */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_IF_MM_EQ_GAIN_ALPHA_QCH_RX4_MASK   (0x00003FFFU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_IF_MM_EQ_GAIN_ALPHA_QCH_RX4_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_IF_MM_EQ_GAIN_ALPHA_QCH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_IF_MM_EQ_GAIN_ALPHA_QCH_RX4_MAX    (0x00003FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU1_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU1_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU3_MASK                           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU3_SHIFT                          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU3_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU4_MASK                           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU4_SHIFT                          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU4_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU5_MASK                           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU5_SHIFT                          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU5_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU6_MASK                           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU6_SHIFT                          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU6_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU6_MAX                            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU2_MASK                           (0xFFF00000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU2_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_NU2_MAX                            (0x00000FFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYP_QCH_RX4_RESETVAL                           (0x00000000U)

/* IF_MM_LPF_HPF_BYP */

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX1_MASK           (0x00000001U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX1_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX1_MASK           (0x00000002U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX1_SHIFT          (0x00000001U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX1_MASK           (0x00000004U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX1_SHIFT          (0x00000002U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX1_MASK           (0x00000008U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX1_SHIFT          (0x00000003U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX2_MASK           (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX2_SHIFT          (0x00000004U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX2_MASK           (0x00000020U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX2_SHIFT          (0x00000005U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX2_MASK           (0x00000040U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX2_SHIFT          (0x00000006U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX2_MASK           (0x00000080U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX2_SHIFT          (0x00000007U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX3_MASK           (0x00000100U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX3_SHIFT          (0x00000008U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX3_MASK           (0x00000200U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX3_SHIFT          (0x00000009U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX3_MASK           (0x00000400U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX3_SHIFT          (0x0000000AU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX3_MASK           (0x00000800U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX3_SHIFT          (0x0000000BU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX4_MASK           (0x00001000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX4_SHIFT          (0x0000000CU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_ICH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX4_MASK           (0x00002000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX4_SHIFT          (0x0000000DU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_ICH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX4_MASK           (0x00004000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX4_SHIFT          (0x0000000EU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_ICH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX4_MASK           (0x00008000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX4_SHIFT          (0x0000000FU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_ICH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX1_MASK           (0x00010000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX1_SHIFT          (0x00000010U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX1_MASK           (0x00020000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX1_SHIFT          (0x00000011U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX1_MASK           (0x00040000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX1_SHIFT          (0x00000012U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX1_MASK           (0x00080000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX1_SHIFT          (0x00000013U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX1_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX1_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX2_MASK           (0x00100000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX2_SHIFT          (0x00000014U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX2_MASK           (0x00200000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX2_SHIFT          (0x00000015U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX2_MASK           (0x00400000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX2_SHIFT          (0x00000016U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX2_MASK           (0x00800000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX2_SHIFT          (0x00000017U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX2_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX2_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX3_MASK           (0x01000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX3_SHIFT          (0x00000018U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX3_MASK           (0x02000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX3_SHIFT          (0x00000019U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX3_MASK           (0x04000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX3_SHIFT          (0x0000001AU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX3_MASK           (0x08000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX3_SHIFT          (0x0000001BU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX3_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX3_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX4_MASK           (0x10000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX4_SHIFT          (0x0000001CU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF1_QCH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX4_MASK           (0x20000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX4_SHIFT          (0x0000001DU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_HPF2_QCH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX4_MASK           (0x40000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX4_SHIFT          (0x0000001EU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF1_QCH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX4_MASK           (0x80000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX4_SHIFT          (0x0000001FU)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX4_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_IF_MM_EQ_BYP_LPF2_QCH_RX4_MAX            (0x00000001U)

#define CSL_BSS_DFE_IF_MM_LPF_HPF_BYP_RESETVAL                                 (0x00000000U)

/* IF_MM_ALPHA_BYPASS_CTRL */

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX1_MASK    (0x00000001U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX1_SHIFT   (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX1_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX2_MASK    (0x00000002U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX2_SHIFT   (0x00000001U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX2_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX3_MASK    (0x00000004U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX3_SHIFT   (0x00000002U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX3_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX4_MASK    (0x00000008U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX4_SHIFT   (0x00000003U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_ICH_RX4_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX1_MASK    (0x00000010U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX1_SHIFT   (0x00000004U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX1_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX2_MASK    (0x00000020U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX2_SHIFT   (0x00000005U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX2_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX3_MASK    (0x00000040U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX3_SHIFT   (0x00000006U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX3_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX4_MASK    (0x00000080U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX4_SHIFT   (0x00000007U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_IF_MM_EQ_BYP_ALPHA_QCH_RX4_MAX     (0x00000001U)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_NU_MASK                            (0xFFFFFF00U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_NU_SHIFT                           (0x00000008U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_NU_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_NU_MAX                             (0x00FFFFFFU)

#define CSL_BSS_DFE_IF_MM_ALPHA_BYPASS_CTRL_RESETVAL                           (0x00000000U)

/* RESAMP_A8_COEFF */

#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_VAL_MASK                        (0x0000FFFFU)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_VAL_SHIFT                       (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_VAL_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_VAL_MAX                         (0x0000FFFFU)

#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_INITVAL_MASK                    (0xFFFF0000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_INITVAL_SHIFT                   (0x00000010U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_INITVAL_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESAMP_TAU_INITVAL_MAX                     (0x0000FFFFU)

#define CSL_BSS_DFE_RESAMP_A8_COEFF_RESETVAL                                   (0x00000000U)

/* RESAMP_A8_COEFF_BYP_CFG */

#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_RESAMP_BYPASS_MASK                 (0x00000001U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_RESAMP_BYPASS_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_RESAMP_BYPASS_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_RESAMP_BYPASS_MAX                  (0x00000001U)

#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_NU_MASK                            (0xFFFFFFFEU)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_NU_SHIFT                           (0x00000001U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_NU_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_NU_MAX                             (0x7FFFFFFFU)

#define CSL_BSS_DFE_RESAMP_A8_COEFF_BYP_CFG_RESETVAL                           (0x00000000U)

/* DC_CORR_RX1 */

#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_ICH_RX1_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_ICH_RX1_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_ICH_RX1_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_ICH_RX1_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_QCH_RX1_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_QCH_RX1_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_QCH_RX1_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX1_DC_CORR_VAL_QCH_RX1_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX1_RESETVAL                                       (0x00000000U)

/* DC_CORR_RX2 */

#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_ICH_RX2_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_ICH_RX2_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_ICH_RX2_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_ICH_RX2_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_QCH_RX2_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_QCH_RX2_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_QCH_RX2_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX2_DC_CORR_VAL_QCH_RX2_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX2_RESETVAL                                       (0x00000000U)

/* DC_CORR_RX3 */

#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_ICH_RX3_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_ICH_RX3_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_ICH_RX3_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_ICH_RX3_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_QCH_RX3_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_QCH_RX3_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_QCH_RX3_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX3_DC_CORR_VAL_QCH_RX3_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX3_RESETVAL                                       (0x00000000U)

/* DC_CORR_RX4 */

#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_ICH_RX4_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_ICH_RX4_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_ICH_RX4_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_ICH_RX4_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_QCH_RX4_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_QCH_RX4_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_QCH_RX4_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_DC_CORR_RX4_DC_CORR_VAL_QCH_RX4_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_DC_CORR_RX4_RESETVAL                                       (0x00000000U)

/* RX_IQMM_CORR_RX1 */

#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_GAIN_CORR_VAL_RX1_MASK        (0x00001FFFU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_GAIN_CORR_VAL_RX1_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_GAIN_CORR_VAL_RX1_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_GAIN_CORR_VAL_RX1_MAX         (0x00001FFFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU1_MASK                                  (0x0000E000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU1_SHIFT                                 (0x0000000DU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU1_MAX                                   (0x00000007U)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_PHASE_CORR_VAL_RX1_MASK       (0x07FF0000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_PHASE_CORR_VAL_RX1_SHIFT      (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_PHASE_CORR_VAL_RX1_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RXIQMM_INIT_PHASE_CORR_VAL_RX1_MAX        (0x000007FFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU2_MASK                                  (0xF8000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU2_SHIFT                                 (0x0000001BU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_NU2_MAX                                   (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX1_RESETVAL                                  (0x00000000U)

/* RX_IQMM_CORR_RX2 */

#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_GAIN_CORR_VAL_RX2_MASK        (0x00001FFFU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_GAIN_CORR_VAL_RX2_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_GAIN_CORR_VAL_RX2_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_GAIN_CORR_VAL_RX2_MAX         (0x00001FFFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU1_MASK                                  (0x0000E000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU1_SHIFT                                 (0x0000000DU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU1_MAX                                   (0x00000007U)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_PHASE_CORR_VAL_RX2_MASK       (0x07FF0000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_PHASE_CORR_VAL_RX2_SHIFT      (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_PHASE_CORR_VAL_RX2_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RXIQMM_INIT_PHASE_CORR_VAL_RX2_MAX        (0x000007FFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU2_MASK                                  (0xF8000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU2_SHIFT                                 (0x0000001BU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_NU2_MAX                                   (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX2_RESETVAL                                  (0x00000000U)

/* RX_IQMM_CORR_RX3 */

#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_GAIN_CORR_VAL_RX3_MASK        (0x00001FFFU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_GAIN_CORR_VAL_RX3_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_GAIN_CORR_VAL_RX3_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_GAIN_CORR_VAL_RX3_MAX         (0x00001FFFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU1_MASK                                  (0x0000E000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU1_SHIFT                                 (0x0000000DU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU1_MAX                                   (0x00000007U)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_PHASE_CORR_VAL_RX3_MASK       (0x07FF0000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_PHASE_CORR_VAL_RX3_SHIFT      (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_PHASE_CORR_VAL_RX3_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RXIQMM_INIT_PHASE_CORR_VAL_RX3_MAX        (0x000007FFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU2_MASK                                  (0xF8000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU2_SHIFT                                 (0x0000001BU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_NU2_MAX                                   (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX3_RESETVAL                                  (0x00000000U)

/* RX_IQMM_CORR_RX4 */

#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_GAIN_CORR_VAL_RX4_MASK        (0x00001FFFU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_GAIN_CORR_VAL_RX4_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_GAIN_CORR_VAL_RX4_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_GAIN_CORR_VAL_RX4_MAX         (0x00001FFFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU1_MASK                                  (0x0000E000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU1_SHIFT                                 (0x0000000DU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU1_MAX                                   (0x00000007U)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_PHASE_CORR_VAL_RX4_MASK       (0x07FF0000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_PHASE_CORR_VAL_RX4_SHIFT      (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_PHASE_CORR_VAL_RX4_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RXIQMM_INIT_PHASE_CORR_VAL_RX4_MAX        (0x000007FFU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU2_MASK                                  (0xF8000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU2_SHIFT                                 (0x0000001BU)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_NU2_MAX                                   (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CORR_RX4_RESETVAL                                  (0x00000000U)

/* AN_UPN_FREQ_CFG_RX1 */

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_AN_UPN_FREQ_SHIFT_WORD_RX1_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_AN_UPN_FREQ_SHIFT_WORD_RX1_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_AN_UPN_FREQ_SHIFT_WORD_RX1_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_AN_UPN_FREQ_SHIFT_WORD_RX1_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX1_RESETVAL                               (0x00000000U)

/* AN_UPN_FREQ_CFG_RX2 */

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_AN_UPN_FREQ_SHIFT_WORD_RX2_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_AN_UPN_FREQ_SHIFT_WORD_RX2_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_AN_UPN_FREQ_SHIFT_WORD_RX2_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_AN_UPN_FREQ_SHIFT_WORD_RX2_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX2_RESETVAL                               (0x00000000U)

/* AN_UPN_FREQ_CFG_RX3 */

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_AN_UPN_FREQ_SHIFT_WORD_RX3_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_AN_UPN_FREQ_SHIFT_WORD_RX3_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_AN_UPN_FREQ_SHIFT_WORD_RX3_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_AN_UPN_FREQ_SHIFT_WORD_RX3_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX3_RESETVAL                               (0x00000000U)

/* AN_UPN_FREQ_CFG_RX4 */

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_AN_UPN_FREQ_SHIFT_WORD_RX4_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_AN_UPN_FREQ_SHIFT_WORD_RX4_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_AN_UPN_FREQ_SHIFT_WORD_RX4_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_AN_UPN_FREQ_SHIFT_WORD_RX4_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_CFG_RX4_RESETVAL                               (0x00000000U)

/* AN_UPN_FREQ_INIT_CFG_RX1 */

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX1_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX1_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX1_RESETVAL                          (0x00000000U)

/* AN_UPN_FREQ_INIT_CFG_RX2 */

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX2_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX2_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX2_RESETVAL                          (0x00000000U)

/* AN_UPN_FREQ_INIT_CFG_RX3 */

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX3_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX3_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX3_RESETVAL                          (0x00000000U)

/* AN_UPN_FREQ_INIT_CFG_RX4 */

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX4_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_AN_UPN_FREQ_SHIFT_INIT_PHASE_RX4_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_AN_UPN_FREQ_INIT_CFG_RX4_RESETVAL                          (0x00000000U)

/* CENTERING_IMJREJ_FREQ_CFG_RX1 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_WORD_RX1_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_WORD_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_WORD_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_WORD_RX1_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_NU_MASK                      (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_NU_SHIFT                     (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_NU_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_NU_MAX                       (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX1_RESETVAL                     (0x00000000U)

/* CENTERING_IMJREJ_FREQ_CFG_RX2 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_WORD_RX2_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_WORD_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_WORD_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_WORD_RX2_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_NU_MASK                      (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_NU_SHIFT                     (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_NU_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_NU_MAX                       (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX2_RESETVAL                     (0x00000000U)

/* CENTERING_IMJREJ_FREQ_CFG_RX3 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_WORD_RX3_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_WORD_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_WORD_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_WORD_RX3_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_NU_MASK                      (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_NU_SHIFT                     (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_NU_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_NU_MAX                       (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX3_RESETVAL                     (0x00000000U)

/* CENTERING_IMJREJ_FREQ_CFG_RX4 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_WORD_RX4_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_WORD_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_WORD_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_WORD_RX4_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_NU_MASK                      (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_NU_SHIFT                     (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_NU_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_NU_MAX                       (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_CFG_RX4_RESETVAL                     (0x00000000U)

/* CENTERING_IMJREJ_FREQ_INIT_CFG_RX1 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX1_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX1_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_NU_MASK                 (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_NU_SHIFT                (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_NU_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_NU_MAX                  (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX1_RESETVAL                (0x00000000U)

/* CENTERING_IMJREJ_FREQ_INIT_CFG_RX2 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX2_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX2_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_NU_MASK                 (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_NU_SHIFT                (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_NU_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_NU_MAX                  (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX2_RESETVAL                (0x00000000U)

/* CENTERING_IMJREJ_FREQ_INIT_CFG_RX3 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX3_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX3_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_NU_MASK                 (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_NU_SHIFT                (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_NU_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_NU_MAX                  (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX3_RESETVAL                (0x00000000U)

/* CENTERING_IMJREJ_FREQ_INIT_CFG_RX4 */

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX4_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_CENT_IMJREJ_FREQ_SHIFT_INIT_PHASE_RX4_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_NU_MASK                 (0xF0000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_NU_SHIFT                (0x0000001CU)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_NU_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_NU_MAX                  (0x0000000FU)

#define CSL_BSS_DFE_CENTERING_IMJREJ_FREQ_INIT_CFG_RX4_RESETVAL                (0x00000000U)

/* DECENT_FREQ_CFG_RX1 */

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_DECENT_FREQ_SHIFT_WORD_RX1_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_DECENT_FREQ_SHIFT_WORD_RX1_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_DECENT_FREQ_SHIFT_WORD_RX1_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_DECENT_FREQ_SHIFT_WORD_RX1_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX1_RESETVAL                               (0x00000000U)

/* DECENT_FREQ_CFG_RX2 */

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_DECENT_FREQ_SHIFT_WORD_RX2_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_DECENT_FREQ_SHIFT_WORD_RX2_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_DECENT_FREQ_SHIFT_WORD_RX2_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_DECENT_FREQ_SHIFT_WORD_RX2_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX2_RESETVAL                               (0x00000000U)

/* DECENT_FREQ_CFG_RX3 */

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_DECENT_FREQ_SHIFT_WORD_RX3_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_DECENT_FREQ_SHIFT_WORD_RX3_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_DECENT_FREQ_SHIFT_WORD_RX3_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_DECENT_FREQ_SHIFT_WORD_RX3_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX3_RESETVAL                               (0x00000000U)

/* DECENT_FREQ_CFG_RX4 */

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_DECENT_FREQ_SHIFT_WORD_RX4_MASK        (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_DECENT_FREQ_SHIFT_WORD_RX4_SHIFT       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_DECENT_FREQ_SHIFT_WORD_RX4_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_DECENT_FREQ_SHIFT_WORD_RX4_MAX         (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_NU_MASK                                (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_CFG_RX4_RESETVAL                               (0x00000000U)

/* DECENT_FREQ_INIT_CFG_RX1 */

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_DECENT_FREQ_SHIFT_INIT_PHASE_RX1_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_DECENT_FREQ_SHIFT_INIT_PHASE_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_DECENT_FREQ_SHIFT_INIT_PHASE_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_DECENT_FREQ_SHIFT_INIT_PHASE_RX1_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX1_RESETVAL                          (0x00000000U)

/* DECENT_FREQ_INIT_CFG_RX2 */

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_DECENT_FREQ_SHIFT_INIT_PHASE_RX2_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_DECENT_FREQ_SHIFT_INIT_PHASE_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_DECENT_FREQ_SHIFT_INIT_PHASE_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_DECENT_FREQ_SHIFT_INIT_PHASE_RX2_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX2_RESETVAL                          (0x00000000U)

/* DECENT_FREQ_INIT_CFG_RX3 */

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_DECENT_FREQ_SHIFT_INIT_PHASE_RX3_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_DECENT_FREQ_SHIFT_INIT_PHASE_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_DECENT_FREQ_SHIFT_INIT_PHASE_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_DECENT_FREQ_SHIFT_INIT_PHASE_RX3_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX3_RESETVAL                          (0x00000000U)

/* DECENT_FREQ_INIT_CFG_RX4 */

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_DECENT_FREQ_SHIFT_INIT_PHASE_RX4_MASK (0x0FFFFFFFU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_DECENT_FREQ_SHIFT_INIT_PHASE_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_DECENT_FREQ_SHIFT_INIT_PHASE_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_DECENT_FREQ_SHIFT_INIT_PHASE_RX4_MAX (0x0FFFFFFFU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_NU_MASK                           (0xF0000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_NU_SHIFT                          (0x0000001CU)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_NU_MAX                            (0x0000000FU)

#define CSL_BSS_DFE_DECENT_FREQ_INIT_CFG_RX4_RESETVAL                          (0x00000000U)

/* DFE_OUTPUT_MODE_SELECTOR */

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M8_SEL_MASK                   (0x00000001U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M8_SEL_SHIFT                  (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M8_SEL_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M8_SEL_MAX                    (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M9_SEL_MASK                   (0x00000002U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M9_SEL_SHIFT                  (0x00000001U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M9_SEL_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M9_SEL_MAX                    (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M10_SEL_MASK                  (0x00000004U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M10_SEL_SHIFT                 (0x00000002U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M10_SEL_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M10_SEL_MAX                   (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M11_SEL_MASK                  (0x00000018U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M11_SEL_SHIFT                 (0x00000003U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M11_SEL_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M11_SEL_MAX                   (0x00000003U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_FOR_AN_UPN_SELECT_MASK (0x00000020U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_FOR_AN_UPN_SELECT_SHIFT (0x00000005U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_FOR_AN_UPN_SELECT_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_FOR_AN_UPN_SELECT_MAX (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_BEFORE_M11_SELECT_MASK (0x00000040U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_BEFORE_M11_SELECT_SHIFT (0x00000006U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_BEFORE_M11_SELECT_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_REAL_OR_IMAG_BEFORE_M11_SELECT_MAX (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_A10_OSR_SELECT_MASK               (0x00000080U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_A10_OSR_SELECT_SHIFT              (0x00000007U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_A10_OSR_SELECT_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_A10_OSR_SELECT_MAX                (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M12_SEL_MASK                  (0x00000100U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M12_SEL_SHIFT                 (0x00000008U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M12_SEL_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_MUX_M12_SEL_MAX                   (0x00000001U)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_NU_MASK                           (0xFFFFFE00U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_NU_SHIFT                          (0x00000009U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_NU_MAX                            (0x007FFFFFU)

#define CSL_BSS_DFE_DFE_OUTPUT_MODE_SELECTOR_RESETVAL                          (0x00000000U)

/* BUMPER_SIG_CFG */

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_CAPT_ENABLE_MASK                   (0x00000001U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_CAPT_ENABLE_SHIFT                  (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_CAPT_ENABLE_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_CAPT_ENABLE_MAX                    (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_ENABLE_MASK          (0x00000004U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_ENABLE_SHIFT         (0x00000002U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_ENABLE_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_ENABLE_MAX           (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_NUM_CHIRPS_IN_FRAME_MASK           (0x00000038U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_NUM_CHIRPS_IN_FRAME_SHIFT          (0x00000003U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_NUM_CHIRPS_IN_FRAME_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_NUM_CHIRPS_IN_FRAME_MAX            (0x00000007U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_NUM_SAMP_PER_CHIRP_MASK (0x00000FC0U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_NUM_SAMP_PER_CHIRP_SHIFT (0x00000006U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_NUM_SAMP_PER_CHIRP_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_FILTERED_CAPT_NUM_SAMP_PER_CHIRP_MAX (0x0000003FU)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_NUM_SAMP_PER_CHIRP_MASK   (0x0007F000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_NUM_SAMP_PER_CHIRP_SHIFT  (0x0000000CU)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_NUM_SAMP_PER_CHIRP_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_RAW_CAPT_NUM_SAMP_PER_CHIRP_MAX    (0x0000007FU)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_DECIM_PHASE_SELECT_MASK            (0x00080000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_DECIM_PHASE_SELECT_SHIFT           (0x00000013U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_DECIM_PHASE_SELECT_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_DECIM_PHASE_SELECT_MAX             (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_ACCESS_DFE_MASK                (0x00100000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_ACCESS_DFE_SHIFT               (0x00000014U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_ACCESS_DFE_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_ACCESS_DFE_MAX                 (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_MASK                      (0x00200000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_SHIFT                     (0x00000015U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_MAX                       (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_DONE_MASK                 (0x00400000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_DONE_SHIFT                (0x00000016U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_DONE_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_BUMP_SIG_MEM_INIT_DONE_MAX                  (0x00000001U)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_NU_MASK                                     (0xFF800000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_NU_SHIFT                                    (0x00000017U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_BUMPER_SIG_CFG_NU_MAX                                      (0x000001FFU)

#define CSL_BSS_DFE_BUMPER_SIG_CFG_RESETVAL                                    (0x00000000U)

/* AGC_IQMM */

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_WINDOWLEN_MASK                         (0x0000001FU)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_WINDOWLEN_SHIFT                        (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_WINDOWLEN_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_WINDOWLEN_MAX                          (0x0000001FU)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_TRIGGER_MASK                           (0x00000020U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_TRIGGER_SHIFT                          (0x00000005U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_TRIGGER_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_TRIGGER_MAX                            (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ONE_SHOT_MODE_ENABLE_MASK              (0x00000040U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ONE_SHOT_MODE_ENABLE_SHIFT             (0x00000006U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ONE_SHOT_MODE_ENABLE_RESETVAL          (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ONE_SHOT_MODE_ENABLE_MAX               (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ACCDONE_MASK                           (0x00000080U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ACCDONE_SHIFT                          (0x00000007U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ACCDONE_RESETVAL                       (0x00000001U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_ACCDONE_MAX                            (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MODE_MASK                              (0x00000100U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MODE_SHIFT                             (0x00000008U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MODE_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MODE_MAX                               (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_MASK                          (0x00000200U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_SHIFT                         (0x00000009U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_MAX                           (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_DONE_MASK                     (0x00000400U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_DONE_SHIFT                    (0x0000000AU)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_DONE_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_STATS_COLL_MEM_INIT_DONE_MAX                      (0x00000001U)

#define CSL_BSS_DFE_AGC_IQMM_NU_MASK                                           (0xFFFFF800U)
#define CSL_BSS_DFE_AGC_IQMM_NU_SHIFT                                          (0x0000000BU)
#define CSL_BSS_DFE_AGC_IQMM_NU_RESETVAL                                       (0x00000000U)
#define CSL_BSS_DFE_AGC_IQMM_NU_MAX                                            (0x001FFFFFU)

#define CSL_BSS_DFE_AGC_IQMM_RESETVAL                                          (0x00000080U)

/* WIDE_BAND_INTF_1 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_INTERF_MON_ENABLE_MASK          (0x00000001U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_INTERF_MON_ENABLE_SHIFT         (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_INTERF_MON_ENABLE_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_INTERF_MON_ENABLE_MAX           (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_HALF_RATE_MODE_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_HALF_RATE_MODE_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_HALF_RATE_MODE_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_HALF_RATE_MODE_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_RX_SELECT_FOR_MONITORING_MASK             (0x0000000CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_RX_SELECT_FOR_MONITORING_SHIFT            (0x00000002U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_RX_SELECT_FOR_MONITORING_RESETVAL         (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_RX_SELECT_FOR_MONITORING_MAX              (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_REAL_ONLY_MODE_MASK                       (0x00000010U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_REAL_ONLY_MODE_SHIFT                      (0x00000004U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_REAL_ONLY_MODE_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_REAL_ONLY_MODE_MAX                        (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_TIME_SLICES_TO_MONITOR_MASK        (0x00000FE0U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_TIME_SLICES_TO_MONITOR_SHIFT       (0x00000005U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_TIME_SLICES_TO_MONITOR_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_TIME_SLICES_TO_MONITOR_MAX         (0x0000007FU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_B1_DECIM_PHASE_SELECT_MASK                (0x00001000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_B1_DECIM_PHASE_SELECT_SHIFT               (0x0000000CU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_B1_DECIM_PHASE_SELECT_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_B1_DECIM_PHASE_SELECT_MAX                 (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG1_DECIM_PHASE_SELECT_MASK          (0x00002000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG1_DECIM_PHASE_SELECT_SHIFT         (0x0000000DU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG1_DECIM_PHASE_SELECT_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG1_DECIM_PHASE_SELECT_MAX           (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG2_DECIM_PHASE_SELECT_MASK          (0x00004000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG2_DECIM_PHASE_SELECT_SHIFT         (0x0000000EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG2_DECIM_PHASE_SELECT_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_QMF_STG2_DECIM_PHASE_SELECT_MAX           (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_PING_PONG_SEL_INVERSION_MASK    (0x00008000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_PING_PONG_SEL_INVERSION_SHIFT   (0x0000000FU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_PING_PONG_SEL_INVERSION_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WIDE_BAND_PING_PONG_SEL_INVERSION_MAX     (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_SAMPLES_PER_TIME_SLICE_MASK        (0x7FFF0000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_SAMPLES_PER_TIME_SLICE_SHIFT       (0x00000010U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_SAMPLES_PER_TIME_SLICE_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_WB_NUM_SAMPLES_PER_TIME_SLICE_MAX         (0x00007FFFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_NU2_MASK                                  (0x80000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_NU2_SHIFT                                 (0x0000001FU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_NU2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_1_NU2_MAX                                   (0x00000001U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_1_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_2 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_2_ADC_NOISE_COMPENSATION_TERM_REGION_R0_MASK (0xFFFFFFFFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_2_ADC_NOISE_COMPENSATION_TERM_REGION_R0_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_2_ADC_NOISE_COMPENSATION_TERM_REGION_R0_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_2_ADC_NOISE_COMPENSATION_TERM_REGION_R0_MAX (0xFFFFFFFFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_2_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_3 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_3_ADC_NOISE_COMPENSATION_TERM_REGION_R1_MASK (0xFFFFFFFFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_3_ADC_NOISE_COMPENSATION_TERM_REGION_R1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_3_ADC_NOISE_COMPENSATION_TERM_REGION_R1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_3_ADC_NOISE_COMPENSATION_TERM_REGION_R1_MAX (0xFFFFFFFFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_3_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_4 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_4_ADC_NOISE_COMPENSATION_TERM_REGION_R2_MASK (0xFFFFFFFFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_4_ADC_NOISE_COMPENSATION_TERM_REGION_R2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_4_ADC_NOISE_COMPENSATION_TERM_REGION_R2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_4_ADC_NOISE_COMPENSATION_TERM_REGION_R2_MAX (0xFFFFFFFFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_4_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_5 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_5_ADC_NOISE_COMPENSATION_TERM_REGION_R3_MASK (0xFFFFFFFFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_5_ADC_NOISE_COMPENSATION_TERM_REGION_R3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_5_ADC_NOISE_COMPENSATION_TERM_REGION_R3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_5_ADC_NOISE_COMPENSATION_TERM_REGION_R3_MAX (0xFFFFFFFFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_5_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_6 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_6_LOW_POWER_THRESHOLD_REGION_R0_PLUS_MASK   (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_LOW_POWER_THRESHOLD_REGION_R0_PLUS_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_LOW_POWER_THRESHOLD_REGION_R0_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_LOW_POWER_THRESHOLD_REGION_R0_PLUS_MAX    (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_6_MID_POWER_THRESHOLD_REGION_R0_PLUS_MASK   (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_MID_POWER_THRESHOLD_REGION_R0_PLUS_SHIFT  (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_MID_POWER_THRESHOLD_REGION_R0_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_MID_POWER_THRESHOLD_REGION_R0_PLUS_MAX    (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_6_HIGH_POWER_THRESHOLD_REGION_R0_PLUS_MASK  (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_HIGH_POWER_THRESHOLD_REGION_R0_PLUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_HIGH_POWER_THRESHOLD_REGION_R0_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_HIGH_POWER_THRESHOLD_REGION_R0_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_6_NU_MASK                                   (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_NU_SHIFT                                  (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_6_NU_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_6_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_7 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_7_LOW_POWER_THRESHOLD_REGION_R0_MINUS_MASK  (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_LOW_POWER_THRESHOLD_REGION_R0_MINUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_LOW_POWER_THRESHOLD_REGION_R0_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_LOW_POWER_THRESHOLD_REGION_R0_MINUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_7_MID_POWER_THRESHOLD_REGION_R0_MINUS_MASK  (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_MID_POWER_THRESHOLD_REGION_R0_MINUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_MID_POWER_THRESHOLD_REGION_R0_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_MID_POWER_THRESHOLD_REGION_R0_MINUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_7_HIGH_POWER_THRESHOLD_REGION_R0_MINUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_HIGH_POWER_THRESHOLD_REGION_R0_MINUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_HIGH_POWER_THRESHOLD_REGION_R0_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_HIGH_POWER_THRESHOLD_REGION_R0_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_7_NU_MASK                                   (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_NU_SHIFT                                  (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_7_NU_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_7_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_8 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_8_LOW_POWER_THRESHOLD_REGION_R1_PLUS_MASK   (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_LOW_POWER_THRESHOLD_REGION_R1_PLUS_SHIFT  (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_LOW_POWER_THRESHOLD_REGION_R1_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_LOW_POWER_THRESHOLD_REGION_R1_PLUS_MAX    (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_8_MID_POWER_THRESHOLD_REGION_R1_PLUS_MASK   (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_MID_POWER_THRESHOLD_REGION_R1_PLUS_SHIFT  (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_MID_POWER_THRESHOLD_REGION_R1_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_MID_POWER_THRESHOLD_REGION_R1_PLUS_MAX    (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_8_HIGH_POWER_THRESHOLD_REGION_R1_PLUS_MASK  (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_HIGH_POWER_THRESHOLD_REGION_R1_PLUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_HIGH_POWER_THRESHOLD_REGION_R1_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_HIGH_POWER_THRESHOLD_REGION_R1_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_8_NU_MASK                                   (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_NU_SHIFT                                  (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_8_NU_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_8_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_9 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_9_LOW_POWER_THRESHOLD_REGION_R1_MINUS_MASK  (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_LOW_POWER_THRESHOLD_REGION_R1_MINUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_LOW_POWER_THRESHOLD_REGION_R1_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_LOW_POWER_THRESHOLD_REGION_R1_MINUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_9_MID_POWER_THRESHOLD_REGION_R1_MINUS_MASK  (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_MID_POWER_THRESHOLD_REGION_R1_MINUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_MID_POWER_THRESHOLD_REGION_R1_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_MID_POWER_THRESHOLD_REGION_R1_MINUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_9_HIGH_POWER_THRESHOLD_REGION_R1_MINUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_HIGH_POWER_THRESHOLD_REGION_R1_MINUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_HIGH_POWER_THRESHOLD_REGION_R1_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_HIGH_POWER_THRESHOLD_REGION_R1_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_9_NU_MASK                                   (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_NU_SHIFT                                  (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_9_NU_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_9_RESETVAL                                  (0x00000000U)

/* WIDE_BAND_INTF_10 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_10_LOW_POWER_THRESHOLD_REGION_R2_PLUS_MASK  (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_LOW_POWER_THRESHOLD_REGION_R2_PLUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_LOW_POWER_THRESHOLD_REGION_R2_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_LOW_POWER_THRESHOLD_REGION_R2_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_10_MID_POWER_THRESHOLD_REGION_R2_PLUS_MASK  (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_MID_POWER_THRESHOLD_REGION_R2_PLUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_MID_POWER_THRESHOLD_REGION_R2_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_MID_POWER_THRESHOLD_REGION_R2_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_10_HIGH_POWER_THRESHOLD_REGION_R2_PLUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_HIGH_POWER_THRESHOLD_REGION_R2_PLUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_HIGH_POWER_THRESHOLD_REGION_R2_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_HIGH_POWER_THRESHOLD_REGION_R2_PLUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_10_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_10_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_10_RESETVAL                                 (0x00000000U)

/* WIDE_BAND_INTF_11 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_11_LOW_POWER_THRESHOLD_REGION_R2_MINUS_MASK (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_LOW_POWER_THRESHOLD_REGION_R2_MINUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_LOW_POWER_THRESHOLD_REGION_R2_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_LOW_POWER_THRESHOLD_REGION_R2_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_11_MID_POWER_THRESHOLD_REGION_R2_MINUS_MASK (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_MID_POWER_THRESHOLD_REGION_R2_MINUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_MID_POWER_THRESHOLD_REGION_R2_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_MID_POWER_THRESHOLD_REGION_R2_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_11_HIGH_POWER_THRESHOLD_REGION_R2_MINUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_HIGH_POWER_THRESHOLD_REGION_R2_MINUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_HIGH_POWER_THRESHOLD_REGION_R2_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_HIGH_POWER_THRESHOLD_REGION_R2_MINUS_MAX (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_11_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_11_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_11_RESETVAL                                 (0x00000000U)

/* WIDE_BAND_INTF_12 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_12_LOW_POWER_THRESHOLD_REGION_R3_PLUS_MASK  (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_LOW_POWER_THRESHOLD_REGION_R3_PLUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_LOW_POWER_THRESHOLD_REGION_R3_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_LOW_POWER_THRESHOLD_REGION_R3_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_12_MID_POWER_THRESHOLD_REGION_R3_PLUS_MASK  (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_MID_POWER_THRESHOLD_REGION_R3_PLUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_MID_POWER_THRESHOLD_REGION_R3_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_MID_POWER_THRESHOLD_REGION_R3_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_12_HIGH_POWER_THRESHOLD_REGION_R3_PLUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_HIGH_POWER_THRESHOLD_REGION_R3_PLUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_HIGH_POWER_THRESHOLD_REGION_R3_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_HIGH_POWER_THRESHOLD_REGION_R3_PLUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_12_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_12_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_12_RESETVAL                                 (0x00000000U)

/* WIDE_BAND_INTF_13 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_13_LOW_POWER_THRESHOLD_REGION_R3_MINUS_MASK (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_LOW_POWER_THRESHOLD_REGION_R3_MINUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_LOW_POWER_THRESHOLD_REGION_R3_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_LOW_POWER_THRESHOLD_REGION_R3_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_13_MID_POWER_THRESHOLD_REGION_R3_MINUS_MASK (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_MID_POWER_THRESHOLD_REGION_R3_MINUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_MID_POWER_THRESHOLD_REGION_R3_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_MID_POWER_THRESHOLD_REGION_R3_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_13_HIGH_POWER_THRESHOLD_REGION_R3_MINUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_HIGH_POWER_THRESHOLD_REGION_R3_MINUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_HIGH_POWER_THRESHOLD_REGION_R3_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_HIGH_POWER_THRESHOLD_REGION_R3_MINUS_MAX (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_13_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_13_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_13_RESETVAL                                 (0x00000000U)

/* WIDE_BAND_INTF_14 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_14_LOW_POWER_THRESHOLD_REGION_R4_PLUS_MASK  (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_LOW_POWER_THRESHOLD_REGION_R4_PLUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_LOW_POWER_THRESHOLD_REGION_R4_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_LOW_POWER_THRESHOLD_REGION_R4_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_14_MID_POWER_THRESHOLD_REGION_R4_PLUS_MASK  (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_MID_POWER_THRESHOLD_REGION_R4_PLUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_MID_POWER_THRESHOLD_REGION_R4_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_MID_POWER_THRESHOLD_REGION_R4_PLUS_MAX   (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_14_HIGH_POWER_THRESHOLD_REGION_R4_PLUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_HIGH_POWER_THRESHOLD_REGION_R4_PLUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_HIGH_POWER_THRESHOLD_REGION_R4_PLUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_HIGH_POWER_THRESHOLD_REGION_R4_PLUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_14_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_14_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_14_RESETVAL                                 (0x00000000U)

/* WIDE_BAND_INTF_15 */

#define CSL_BSS_DFE_WIDE_BAND_INTF_15_LOW_POWER_THRESHOLD_REGION_R4_MINUS_MASK (0x000003FFU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_LOW_POWER_THRESHOLD_REGION_R4_MINUS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_LOW_POWER_THRESHOLD_REGION_R4_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_LOW_POWER_THRESHOLD_REGION_R4_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_15_MID_POWER_THRESHOLD_REGION_R4_MINUS_MASK (0x000FFC00U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_MID_POWER_THRESHOLD_REGION_R4_MINUS_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_MID_POWER_THRESHOLD_REGION_R4_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_MID_POWER_THRESHOLD_REGION_R4_MINUS_MAX  (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_15_HIGH_POWER_THRESHOLD_REGION_R4_MINUS_MASK (0x3FF00000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_HIGH_POWER_THRESHOLD_REGION_R4_MINUS_SHIFT (0x00000014U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_HIGH_POWER_THRESHOLD_REGION_R4_MINUS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_HIGH_POWER_THRESHOLD_REGION_R4_MINUS_MAX (0x000003FFU)

#define CSL_BSS_DFE_WIDE_BAND_INTF_15_NU_MASK                                  (0xC0000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_NU_SHIFT                                 (0x0000001EU)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_WIDE_BAND_INTF_15_NU_MAX                                   (0x00000003U)

#define CSL_BSS_DFE_WIDE_BAND_INTF_15_RESETVAL                                 (0x00000000U)

/* SIG_IMG_BAND_1 */

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_MON_ENABLE_MASK                     (0x00000001U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_MON_ENABLE_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_MON_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_MON_ENABLE_MAX                      (0x00000001U)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_TIME_SLICES_TO_MONITOR_MASK     (0x000000FEU)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_TIME_SLICES_TO_MONITOR_SHIFT    (0x00000001U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_TIME_SLICES_TO_MONITOR_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_TIME_SLICES_TO_MONITOR_MAX      (0x0000007FU)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_SAMPLES_PER_TIME_SLICE_MASK     (0x0007FF00U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_SAMPLES_PER_TIME_SLICE_SHIFT    (0x00000008U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_SAMPLES_PER_TIME_SLICE_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_NUM_SAMPLES_PER_TIME_SLICE_MAX      (0x000007FFU)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_GAIN_COMP_DB_MASK                   (0x3FF80000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_GAIN_COMP_DB_SHIFT                  (0x00000013U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_GAIN_COMP_DB_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_GAIN_COMP_DB_MAX                    (0x000007FFU)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_B1_DECIM_PHASE_SELECT_MASK          (0x40000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_B1_DECIM_PHASE_SELECT_SHIFT         (0x0000001EU)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_B1_DECIM_PHASE_SELECT_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_B1_DECIM_PHASE_SELECT_MAX           (0x00000001U)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_QMF_STG1_DECIM_PHASE_SELECT_MASK    (0x80000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_QMF_STG1_DECIM_PHASE_SELECT_SHIFT   (0x0000001FU)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_QMF_STG1_DECIM_PHASE_SELECT_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_1_SIG_IMG_QMF_STG1_DECIM_PHASE_SELECT_MAX     (0x00000001U)

#define CSL_BSS_DFE_SIG_IMG_BAND_1_RESETVAL                                    (0x00000000U)

/* SIG_IMG_BAND_2 */

#define CSL_BSS_DFE_SIG_IMG_BAND_2_ADC_NOISE_COMPENSATION_TERM_SIG_IMG_MASK    (0xFFFFFFFFU)
#define CSL_BSS_DFE_SIG_IMG_BAND_2_ADC_NOISE_COMPENSATION_TERM_SIG_IMG_SHIFT   (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_2_ADC_NOISE_COMPENSATION_TERM_SIG_IMG_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_SIG_IMG_BAND_2_ADC_NOISE_COMPENSATION_TERM_SIG_IMG_MAX     (0xFFFFFFFFU)

#define CSL_BSS_DFE_SIG_IMG_BAND_2_RESETVAL                                    (0x00000000U)

/* RF_RESPOSE_EQ_RX_CFG1 */

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_EQ_TAU_MASK               (0x0000FFFFU)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_EQ_TAU_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_EQ_TAU_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_EQ_TAU_MAX                (0x0000FFFFU)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_TX_SELECT_FOR_TX_COMP_IN_RX_MASK     (0x00030000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_TX_SELECT_FOR_TX_COMP_IN_RX_SHIFT    (0x00000010U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_TX_SELECT_FOR_TX_COMP_IN_RX_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_TX_SELECT_FOR_TX_COMP_IN_RX_MAX      (0x00000003U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_TX_COMP_IN_RX_MASK                (0x00040000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_TX_COMP_IN_RX_SHIFT               (0x00000012U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_TX_COMP_IN_RX_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_TX_COMP_IN_RX_MAX                 (0x00000001U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_RF_RESP_COMP_IN_RX_MASK           (0x00080000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_RF_RESP_COMP_IN_RX_SHIFT          (0x00000013U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_RF_RESP_COMP_IN_RX_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_EN_RF_RESP_COMP_IN_RX_MAX            (0x00000001U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_MEM_ACCESS_DFE_MASK       (0x00100000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_MEM_ACCESS_DFE_SHIFT      (0x00000014U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_MEM_ACCESS_DFE_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RX_RF_RESP_MEM_ACCESS_DFE_MAX        (0x00000001U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_NU_MASK                              (0xFFE00000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_NU_SHIFT                             (0x00000015U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_NU_MAX                               (0x000007FFU)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG1_RESETVAL                             (0x00000000U)

/* RF_RESPOSE_EQ_RX_CFG2 */

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_RF_RESP_EQ_TAU_ACC_INIT_MASK         (0x007FFFFFU)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_RF_RESP_EQ_TAU_ACC_INIT_SHIFT        (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_RF_RESP_EQ_TAU_ACC_INIT_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_RF_RESP_EQ_TAU_ACC_INIT_MAX          (0x007FFFFFU)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_NU_MASK                              (0xFF800000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_NU_SHIFT                             (0x00000017U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_NU_MAX                               (0x000001FFU)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG2_RESETVAL                             (0x00000000U)

/* LFSR_RX_IFMM_ICH_RX1_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX1_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX1_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX1_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_ICH_RX2_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX2_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX2_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX2_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_ICH_RX3_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX3_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX3_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX3_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_ICH_RX4_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX4_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_ICH_RX4_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_ICH_RX4_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_QCH_RX1_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX1_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX1_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX1_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_QCH_RX2_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX2_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX2_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX2_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_QCH_RX3_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX3_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX3_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX3_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_IFMM_QCH_RX4_CFG */

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX4_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_IFMMEQ_LFSR_DITHER_SOURCE_SEED_QCH_RX4_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_IFMM_QCH_RX4_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_FREQ_SHIFT_1_CFG */

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_1_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_1_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_1_CFG_RESETVAL                          (0x00000000U)

/* LFSR_RX_FREQ_SHIFT_2_CFG */

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_2_MASK (0x1FFFFFFFU)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_FREQ_SHIFT_LFSR_DITHER_SOURCE_SEED_2_MAX (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_NU_MASK                           (0xE0000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_NU_SHIFT                          (0x0000001DU)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_NU_MAX                            (0x00000007U)

#define CSL_BSS_DFE_LFSR_RX_FREQ_SHIFT_2_CFG_RESETVAL                          (0x00000000U)

/* DECIM_PHASE_SELECT_CONFIG */

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A1_MASK         (0x00000001U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A1_SHIFT        (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A1_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A1_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A2_MASK         (0x00000002U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A2_SHIFT        (0x00000001U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A2_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A2_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A3_MASK         (0x00000004U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A3_SHIFT        (0x00000002U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A3_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A3_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A4_MASK         (0x00000008U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A4_SHIFT        (0x00000003U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A4_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A4_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A5_MASK         (0x00000010U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A5_SHIFT        (0x00000004U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A5_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A5_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A6_MASK         (0x00000020U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A6_SHIFT        (0x00000005U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A6_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A6_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A7_MASK         (0x00000040U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A7_SHIFT        (0x00000006U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A7_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A7_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A9_MASK         (0x00000080U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A9_SHIFT        (0x00000007U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A9_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_A9_MAX          (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M10_MASK        (0x00000100U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M10_SHIFT       (0x00000008U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M10_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M10_MAX         (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M11_MASK        (0x00000200U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M11_SHIFT       (0x00000009U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M11_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_CFG_PHASE_SELECT_M11_MAX         (0x00000001U)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_NU_MASK                          (0xFFFFFC00U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_NU_SHIFT                         (0x0000000AU)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_NU_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_NU_MAX                           (0x003FFFFFU)

#define CSL_BSS_DFE_DECIM_PHASE_SELECT_CONFIG_RESETVAL                         (0x00000000U)

/* LFSR_LOAD_CONFIG */

#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_IFMMEQ_LFSR_SEED_LOAD_MASK                (0x00000001U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_IFMMEQ_LFSR_SEED_LOAD_SHIFT               (0x00000000U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_IFMMEQ_LFSR_SEED_LOAD_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_IFMMEQ_LFSR_SEED_LOAD_MAX                 (0x00000001U)

#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_FREQ_SHIFT_LFSR_SEED_LOAD_MASK            (0x00000002U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_FREQ_SHIFT_LFSR_SEED_LOAD_SHIFT           (0x00000001U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_FREQ_SHIFT_LFSR_SEED_LOAD_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_FREQ_SHIFT_LFSR_SEED_LOAD_MAX             (0x00000001U)

#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_CLK_GATE_MASK                         (0x00000004U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_CLK_GATE_SHIFT                        (0x00000002U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_CLK_GATE_MAX                          (0x00000001U)

#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_RESERVED_1_MASK                       (0xFFFFFFF8U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_RESERVED_1_SHIFT                      (0x00000003U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_RESERVED_1_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_DFE_RESERVED_1_MAX                        (0x1FFFFFFFU)

#define CSL_BSS_DFE_LFSR_LOAD_CONFIG_RESETVAL                                  (0x00000000U)

/* ADC_VALID_CNTR_STAT_COLL0_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE0_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE0_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE0_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE0_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_NU_MASK                (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_NU_SHIFT               (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_NU_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_NU_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_START_CFG_RESETVAL               (0x00000000U)

/* ADC_VALID_CNTR_STAT_COLL0_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE0_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE0_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE0_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE0_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_NU_MASK                  (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_NU_SHIFT                 (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_NU_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_NU_MAX                   (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL0_END_CFG_RESETVAL                 (0x00000000U)

/* ADC_VALID_CNTR_STAT_COLL1_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE1_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_ADC_VALID_START_TIME_STAT_COLL_MODE1_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_NU_MASK                (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_NU_SHIFT               (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_NU_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_NU_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_START_CFG_RESETVAL               (0x00000000U)

/* ADC_VALID_CNTR_STAT_COLL1_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE1_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_ADC_VALID_END_TIME_STAT_COLL_MODE1_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_NU_MASK                  (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_NU_SHIFT                 (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_NU_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_NU_MAX                   (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_STAT_COLL1_END_CFG_RESETVAL                 (0x00000000U)

/* ADC_VALID_CNTR_BUMP_SIG_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_ADC_VALID_START_TIME_BUMP_SIG_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_ADC_VALID_START_TIME_BUMP_SIG_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_ADC_VALID_START_TIME_BUMP_SIG_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_ADC_VALID_START_TIME_BUMP_SIG_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_NU_MASK                  (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_NU_SHIFT                 (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_NU_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_NU_MAX                   (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_START_CFG_RESETVAL                 (0x00000000U)

/* ADC_VALID_CNTR_BUMP_SIG_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_ADC_VALID_END_TIME_BUMP_SIG_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_ADC_VALID_END_TIME_BUMP_SIG_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_ADC_VALID_END_TIME_BUMP_SIG_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_ADC_VALID_END_TIME_BUMP_SIG_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_NU_MASK                    (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_NU_SHIFT                   (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_NU_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_NU_MAX                     (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_BUMP_SIG_END_CFG_RESETVAL                   (0x00000000U)

/* ADC_VALID_CNTR_WIDE_BAND_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_ADC_VALID_START_TIME_WIDE_BAND_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_ADC_VALID_START_TIME_WIDE_BAND_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_ADC_VALID_START_TIME_WIDE_BAND_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_ADC_VALID_START_TIME_WIDE_BAND_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_NU_MASK                 (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_NU_SHIFT                (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_NU_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_NU_MAX                  (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_START_CFG_RESETVAL                (0x00000000U)

/* ADC_VALID_CNTR_WIDE_BAND_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_ADC_VALID_END_TIME_WIDE_BAND_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_ADC_VALID_END_TIME_WIDE_BAND_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_ADC_VALID_END_TIME_WIDE_BAND_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_ADC_VALID_END_TIME_WIDE_BAND_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_NU_MASK                   (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_NU_SHIFT                  (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_NU_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_NU_MAX                    (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_WIDE_BAND_END_CFG_RESETVAL                  (0x00000000U)

/* ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_ADC_VALID_START_TIME_SIG_IMG_BAND_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_ADC_VALID_START_TIME_SIG_IMG_BAND_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_ADC_VALID_START_TIME_SIG_IMG_BAND_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_ADC_VALID_START_TIME_SIG_IMG_BAND_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_NU_MASK              (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_NU_SHIFT             (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_NU_RESETVAL          (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_NU_MAX               (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_START_CFG_RESETVAL             (0x00000000U)

/* ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_ADC_VALID_END_TIME_SIG_IMG_BAND_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_ADC_VALID_END_TIME_SIG_IMG_BAND_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_ADC_VALID_END_TIME_SIG_IMG_BAND_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_ADC_VALID_END_TIME_SIG_IMG_BAND_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_NU_MASK                (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_NU_SHIFT               (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_NU_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_NU_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_SIG_IMG_BAND_END_CFG_RESETVAL               (0x00000000U)

/* ADC_VALID_CNTR_DFE_OUT_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_ADC_VALID_START_TIME_DFE_OUT_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_ADC_VALID_START_TIME_DFE_OUT_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_ADC_VALID_START_TIME_DFE_OUT_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_ADC_VALID_START_TIME_DFE_OUT_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_NU_MASK                   (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_NU_SHIFT                  (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_NU_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_NU_MAX                    (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_START_CFG_RESETVAL                  (0x00000000U)

/* ADC_VALID_CNTR_DFE_OUT_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_ADC_VALID_END_TIME_DFE_OUT_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_ADC_VALID_END_TIME_DFE_OUT_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_ADC_VALID_END_TIME_DFE_OUT_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_ADC_VALID_END_TIME_DFE_OUT_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_NU_MASK                     (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_NU_SHIFT                    (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_NU_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_NU_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_OUT_END_CFG_RESETVAL                    (0x00000000U)

/* ADC_VALID_CNTR_DFE_IN_START_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_ADC_VALID_START_TIME_DFE_IN_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_ADC_VALID_START_TIME_DFE_IN_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_ADC_VALID_START_TIME_DFE_IN_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_ADC_VALID_START_TIME_DFE_IN_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_NU_MASK                    (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_NU_SHIFT                   (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_NU_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_NU_MAX                     (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_START_CFG_RESETVAL                   (0x00000000U)

/* ADC_VALID_CNTR_DFE_IN_END_CFG */

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_ADC_VALID_END_TIME_DFE_IN_MASK (0x000FFFFFU)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_ADC_VALID_END_TIME_DFE_IN_SHIFT (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_ADC_VALID_END_TIME_DFE_IN_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_ADC_VALID_END_TIME_DFE_IN_MAX (0x000FFFFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_NU_MASK                      (0xFFF00000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_NU_SHIFT                     (0x00000014U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_NU_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_NU_MAX                       (0x00000FFFU)

#define CSL_BSS_DFE_ADC_VALID_CNTR_DFE_IN_END_CFG_RESETVAL                     (0x00000000U)

/* RAMP_GEN_CONFIG */

#define CSL_BSS_DFE_RAMP_GEN_CONFIG_CHIRP_STITCHING_FREEZE_ENABLE_MASK         (0x00000001U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_CHIRP_STITCHING_FREEZE_ENABLE_SHIFT        (0x00000000U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_CHIRP_STITCHING_FREEZE_ENABLE_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_CHIRP_STITCHING_FREEZE_ENABLE_MAX          (0x00000001U)

#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_SEL_MASK                      (0x00000002U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_SEL_SHIFT                     (0x00000001U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_SEL_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_SEL_MAX                       (0x00000001U)

#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_INVERSION_DISABLE_MASK        (0x00000004U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_INVERSION_DISABLE_SHIFT       (0x00000002U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_INVERSION_DISABLE_RESETVAL    (0x00000000U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RAMP_GEN_CLK_INVERSION_DISABLE_MAX         (0x00000001U)

#define CSL_BSS_DFE_RAMP_GEN_CONFIG_NU_MASK                                    (0xFFFFFFF8U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_NU_SHIFT                                   (0x00000003U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_RAMP_GEN_CONFIG_NU_MAX                                     (0x1FFFFFFFU)

#define CSL_BSS_DFE_RAMP_GEN_CONFIG_RESETVAL                                   (0x00000000U)

/* DFE_CHAIN_ENABLES */

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX1_GATE_MASK                            (0x00000001U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX1_GATE_SHIFT                           (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX1_GATE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX1_GATE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX2_GATE_MASK                            (0x00000002U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX2_GATE_SHIFT                           (0x00000001U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX2_GATE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX2_GATE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX3_GATE_MASK                            (0x00000004U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX3_GATE_SHIFT                           (0x00000002U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX3_GATE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX3_GATE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX4_GATE_MASK                            (0x00000008U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX4_GATE_SHIFT                           (0x00000003U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX4_GATE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX4_GATE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_DFE_GATE_MASK                            (0x00000010U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_DFE_GATE_SHIFT                           (0x00000004U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_DFE_GATE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_DFE_GATE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_ICH_GATE_MASK                         (0x00000020U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_ICH_GATE_SHIFT                        (0x00000005U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_ICH_GATE_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_ICH_GATE_MAX                          (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_QCH_GATE_MASK                         (0x00000040U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_QCH_GATE_SHIFT                        (0x00000006U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_QCH_GATE_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RX_QCH_GATE_MAX                          (0x00000001U)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_NU_MASK                                  (0xFFFFFF80U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_NU_SHIFT                                 (0x00000007U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_NU_MAX                                   (0x01FFFFFFU)

#define CSL_BSS_DFE_DFE_CHAIN_ENABLES_RESETVAL                                 (0x00000000U)

/* DFE_PARITY_CFG */

#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_COMP_DIS_MASK                        (0x1FFFFFFFU)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_COMP_DIS_SHIFT                       (0x00000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_COMP_DIS_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_COMP_DIS_MAX                         (0x1FFFFFFFU)

#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_INVERSION_ENABLE_MASK                (0x20000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_INVERSION_ENABLE_SHIFT               (0x0000001DU)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_INVERSION_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_INVERSION_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_OR_OUTPUT_MASK_MASK                  (0x40000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_OR_OUTPUT_MASK_SHIFT                 (0x0000001EU)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_OR_OUTPUT_MASK_RESETVAL              (0x00000001U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_OR_OUTPUT_MASK_MAX                   (0x00000001U)

#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_AND_OUTPUT_MASK_MASK                 (0x80000000U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_AND_OUTPUT_MASK_SHIFT                (0x0000001FU)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_AND_OUTPUT_MASK_RESETVAL             (0x00000001U)
#define CSL_BSS_DFE_DFE_PARITY_CFG_PARITY_AND_OUTPUT_MASK_MAX                  (0x00000001U)

#define CSL_BSS_DFE_DFE_PARITY_CFG_RESETVAL                                    (0xC0000000U)

/* DFE_DEBUG_CFG1 */

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL1_MASK               (0x0000001FU)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL1_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL1_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL1_MAX                (0x0000001FU)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL2_MASK               (0x000003E0U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL2_SHIFT              (0x00000005U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL2_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL2_MAX                (0x0000001FU)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL3_MASK               (0x00003C00U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL3_SHIFT              (0x0000000AU)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL3_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DATA_SELECT_LEVEL3_MAX                (0x0000000FU)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_ADC_CAPTURE_TRIGGER_MASK              (0x00004000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_ADC_CAPTURE_TRIGGER_SHIFT             (0x0000000EU)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_ADC_CAPTURE_TRIGGER_RESETVAL          (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_ADC_CAPTURE_TRIGGER_MAX               (0x00000001U)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_CLK_RST_GEN_TRIGGER_MASK              (0x00008000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_CLK_RST_GEN_TRIGGER_SHIFT             (0x0000000FU)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_CLK_RST_GEN_TRIGGER_RESETVAL          (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_CLK_RST_GEN_TRIGGER_MAX               (0x00000001U)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DFE_FILTER_TRIGGER_MASK               (0xFFFF0000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DFE_FILTER_TRIGGER_SHIFT              (0x00000010U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DFE_FILTER_TRIGGER_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_DFE_DEBUG_CFG1_DEBUG_DFE_FILTER_TRIGGER_MAX                (0x0000FFFFU)

#define CSL_BSS_DFE_DFE_DEBUG_CFG1_RESETVAL                                    (0x00000000U)

/* DFE_DSPSS_INTF_CFG1 */

#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1_DFE_DSPSS_INTF_CFG1_VAL_MASK           (0xFFFFFFFFU)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1_DFE_DSPSS_INTF_CFG1_VAL_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1_DFE_DSPSS_INTF_CFG1_VAL_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1_DFE_DSPSS_INTF_CFG1_VAL_MAX            (0xFFFFFFFFU)

#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG1_RESETVAL                               (0x00000000U)

/* DFE_DSPSS_INTF_CFG2 */

#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2_DFE_DSPSS_INTF_CFG2_VAL_MASK           (0xFFFFFFFFU)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2_DFE_DSPSS_INTF_CFG2_VAL_SHIFT          (0x00000000U)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2_DFE_DSPSS_INTF_CFG2_VAL_RESETVAL       (0x00000000U)
#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2_DFE_DSPSS_INTF_CFG2_VAL_MAX            (0xFFFFFFFFU)

#define CSL_BSS_DFE_DFE_DSPSS_INTF_CFG2_RESETVAL                               (0x00000000U)

/* IFMM_EQ_HPF_ERROR_SIG_SWAP */

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX1_MASK (0x00000001U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX1_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX2_MASK (0x00000002U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX2_SHIFT (0x00000001U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX2_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX3_MASK (0x00000004U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX3_SHIFT (0x00000002U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX3_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX4_MASK (0x00000008U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX4_SHIFT (0x00000003U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF1_ERROR_REG_SWAP_ENABLE_RX4_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU1_MASK                        (0x000000F0U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU1_SHIFT                       (0x00000004U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU1_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU1_MAX                         (0x0000000FU)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX1_MASK (0x00000100U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX1_SHIFT (0x00000008U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX1_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX2_MASK (0x00000200U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX2_SHIFT (0x00000009U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX2_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX3_MASK (0x00000400U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX3_SHIFT (0x0000000AU)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX3_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX4_MASK (0x00000800U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX4_SHIFT (0x0000000BU)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_IFMM_HPF2_ERROR_REG_SWAP_ENABLE_RX4_MAX (0x00000001U)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU2_MASK                        (0xFFFFF000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU2_SHIFT                       (0x0000000CU)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU2_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_NU2_MAX                         (0x000FFFFFU)

#define CSL_BSS_DFE_IFMM_EQ_HPF_ERROR_SIG_SWAP_RESETVAL                        (0x00000000U)

/* CQ_ADC_SATURATION_CFG */

#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_CHANNEL_MASK_MASK     (0x000000FFU)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_CHANNEL_MASK_SHIFT    (0x00000000U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_CHANNEL_MASK_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_CHANNEL_MASK_MAX      (0x000000FFU)

#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_THRES_MASK            (0x0000FF00U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_THRES_SHIFT           (0x00000008U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_THRES_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_ADC_SATURATION_THRES_MAX             (0x000000FFU)

#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_NU_MASK                              (0xFFFF0000U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_NU_SHIFT                             (0x00000010U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_NU_MAX                               (0x0000FFFFU)

#define CSL_BSS_DFE_CQ_ADC_SATURATION_CFG_RESETVAL                             (0x00000000U)

/* DFE_OUTPUT_BIT_SELECTION */

#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SATURATION_BITS_MASK  (0x00000007U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SATURATION_BITS_SHIFT (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SATURATION_BITS_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SATURATION_BITS_MAX   (0x00000007U)

#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SHIFT_MASK            (0x00000078U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SHIFT_SHIFT           (0x00000003U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SHIFT_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_DFE_OUT_NUM_SHIFT_MAX             (0x0000000FU)

#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_NU_MASK                           (0xFFFFFF80U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_NU_SHIFT                          (0x00000007U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_NU_MAX                            (0x01FFFFFFU)

#define CSL_BSS_DFE_DFE_OUTPUT_BIT_SELECTION_RESETVAL                          (0x00000000U)

/* CQ_SPI_MODE_CFG */

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_BAND_MASK_MASK                         (0x000000FFU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_BAND_MASK_SHIFT                        (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_BAND_MASK_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_BAND_MASK_MAX                          (0x000000FFU)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_SPI_MODE_ADDR_RST_MASK      (0x00000100U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_SPI_MODE_ADDR_RST_SHIFT     (0x00000008U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_SPI_MODE_ADDR_RST_RESETVAL  (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_SPI_MODE_ADDR_RST_MAX       (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_MAX_SIZE_MASK               (0x00000200U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_MAX_SIZE_SHIFT              (0x00000009U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_MAX_SIZE_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_WBE_ADCSAT_MEM_MAX_SIZE_MAX                (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_SPI_MODE_ADDR_RST_MASK          (0x00000400U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_SPI_MODE_ADDR_RST_SHIFT         (0x0000000AU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_SPI_MODE_ADDR_RST_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_SPI_MODE_ADDR_RST_MAX           (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_MAX_SIZE_MASK                   (0x00000800U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_MAX_SIZE_SHIFT                  (0x0000000BU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_MAX_SIZE_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_MEM_MAX_SIZE_MAX                    (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_BAND_MASK_MASK                      (0x00003000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_BAND_MASK_SHIFT                     (0x0000000CU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_BAND_MASK_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_BAND_MASK_MAX                       (0x00000003U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_MASK                          (0x00004000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_SHIFT                         (0x0000000EU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_MAX                           (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_MASK                          (0x00008000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_SHIFT                         (0x0000000FU)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_MAX                           (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_DONE_MASK                     (0x00010000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_DONE_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_DONE_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM1_INIT_DONE_MAX                      (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_DONE_MASK                     (0x00020000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_DONE_SHIFT                    (0x00000011U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_DONE_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_CQ_MEM2_INIT_DONE_MAX                      (0x00000001U)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_IMG_BAND_THRES_MASK                 (0x01FC0000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_IMG_BAND_THRES_SHIFT                (0x00000012U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_IMG_BAND_THRES_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_IMG_BAND_THRES_MAX                  (0x0000007FU)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_SIG_BAND_THRES_MASK                 (0xFE000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_SIG_BAND_THRES_SHIFT                (0x00000019U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_SIG_BAND_THRES_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_SIGIMG_SIG_BAND_THRES_MAX                  (0x0000007FU)

#define CSL_BSS_DFE_CQ_SPI_MODE_CFG_RESETVAL                                   (0x00000000U)

/* AGC_ECC_CFG */

#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_ENABLE_MASK                        (0x00000001U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_ENABLE_SHIFT                       (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_ENABLE_MAX                         (0x00000001U)

#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_CLR_MASK                           (0x00000002U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_CLR_SHIFT                          (0x00000001U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_CLR_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_ECC_CLR_MAX                            (0x00000001U)

#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_REPAIRED_BIT_MASK                      (0x000001FCU)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_REPAIRED_BIT_SHIFT                     (0x00000002U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_REPAIRED_BIT_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_REPAIRED_BIT_MAX                       (0x0000007FU)

#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_FAULT_ADDRESS_MASK                     (0x0003FE00U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_FAULT_ADDRESS_SHIFT                    (0x00000009U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_FAULT_ADDRESS_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_AGC_RAM_FAULT_ADDRESS_MAX                      (0x000001FFU)

#define CSL_BSS_DFE_AGC_ECC_CFG_NU_MASK                                        (0xFFFC0000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_NU_SHIFT                                       (0x00000012U)
#define CSL_BSS_DFE_AGC_ECC_CFG_NU_RESETVAL                                    (0x00000000U)
#define CSL_BSS_DFE_AGC_ECC_CFG_NU_MAX                                         (0x00003FFFU)

#define CSL_BSS_DFE_AGC_ECC_CFG_RESETVAL                                       (0x00000000U)

/* FIR_3TAP_CFG */

#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_COEFFT_MASK                          (0x0000000FU)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_COEFFT_SHIFT                         (0x00000000U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_COEFFT_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_COEFFT_MAX                           (0x0000000FU)

#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_SHIFT_VAL_MASK                       (0x000000F0U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_SHIFT_VAL_SHIFT                      (0x00000004U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_SHIFT_VAL_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_SHIFT_VAL_MAX                        (0x0000000FU)

#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_EN_MASK                              (0x00000100U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_EN_SHIFT                             (0x00000008U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_EN_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_FIR_3TAP_EN_MAX                               (0x00000001U)

#define CSL_BSS_DFE_FIR_3TAP_CFG_NU_MASK                                       (0xFFFFFE00U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_NU_SHIFT                                      (0x00000009U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_FIR_3TAP_CFG_NU_MAX                                        (0x007FFFFFU)

#define CSL_BSS_DFE_FIR_3TAP_CFG_RESETVAL                                      (0x00000000U)

/* CQ_CHIRP_PROFILE_CFG1 */

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_WBI_MASK          (0x0000FFFFU)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_WBI_SHIFT         (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_WBI_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_WBI_MAX           (0x0000FFFFU)

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_SIGIMG_MASK       (0xFFFF0000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_SIGIMG_SHIFT      (0x00000010U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_SIGIMG_RESETVAL   (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_CHIRP_PROFILE_MASK_SIGIMG_MAX        (0x0000FFFFU)

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG1_RESETVAL                             (0x00000000U)

/* CQ_CHIRP_PROFILE_CFG2 */

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_CHIRP_PROFILE_MASK_ADC_MASK          (0x0000FFFFU)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_CHIRP_PROFILE_MASK_ADC_SHIFT         (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_CHIRP_PROFILE_MASK_ADC_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_CHIRP_PROFILE_MASK_ADC_MAX           (0x0000FFFFU)

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_NU_MASK                              (0xFFFF0000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_NU_SHIFT                             (0x00000010U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_NU_MAX                               (0x0000FFFFU)

#define CSL_BSS_DFE_CQ_CHIRP_PROFILE_CFG2_RESETVAL                             (0x00000000U)

/* RF_RESPOSE_EQ_RX_CFG3 */

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX0_EN_MASK (0x00000003U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX0_EN_SHIFT (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX0_EN_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX0_EN_MAX (0x00000003U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX1_EN_MASK (0x0000000CU)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX1_EN_SHIFT (0x00000002U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX1_EN_RESETVAL (0x00000001U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX1_EN_MAX (0x00000003U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX2_EN_MASK (0x00000030U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX2_EN_SHIFT (0x00000004U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX2_EN_RESETVAL (0x00000002U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_TX_SEL_FOR_TX_COMP_IN_RX_WHEN_TX2_EN_MAX (0x00000003U)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_NU_MASK                              (0xFFFFFFC0U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_NU_SHIFT                             (0x00000006U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_NU_MAX                               (0x03FFFFFFU)

#define CSL_BSS_DFE_RF_RESPOSE_EQ_RX_CFG3_RESETVAL                             (0x00000024U)

/* TEST_SRC_CTRL */

#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_RX_TEST_SRC_MASK                          (0x00000001U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_RX_TEST_SRC_SHIFT                         (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_RX_TEST_SRC_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_RX_TEST_SRC_MAX                           (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS1_MASK                 (0x00000002U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS1_SHIFT                (0x00000001U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS1_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS1_MAX                  (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS2_MASK                 (0x00000004U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS2_SHIFT                (0x00000002U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS2_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS2_MAX                  (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS3_MASK                 (0x00000008U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS3_SHIFT                (0x00000003U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS3_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_EN_DELTA_FREQ_ACCUM_TS3_MAX                  (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_ENA_PHASE_DITHER_MASK                        (0x00000010U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_ENA_PHASE_DITHER_SHIFT                       (0x00000004U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_ENA_PHASE_DITHER_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_ENA_PHASE_DITHER_MAX                         (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_NU1_MASK                                     (0x00000060U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU1_SHIFT                                    (0x00000005U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU1_MAX                                      (0x00000003U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_RST_TEST_SRC_MASK                            (0x00000080U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_RST_TEST_SRC_SHIFT                           (0x00000007U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_RST_TEST_SRC_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_RST_TEST_SRC_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_DFE_TAPIN_POINT_MASK                         (0x00000100U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DFE_TAPIN_POINT_SHIFT                        (0x00000008U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DFE_TAPIN_POINT_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DFE_TAPIN_POINT_MAX                          (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_NU2_MASK                                     (0x0000FE00U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU2_MAX                                      (0x0000007FU)

#define CSL_BSS_DFE_TEST_SRC_CTRL_DOPPLE_ACC_MODE_MASK                         (0x00010000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DOPPLE_ACC_MODE_SHIFT                        (0x00000010U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DOPPLE_ACC_MODE_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_DOPPLE_ACC_MODE_MAX                          (0x00000001U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_NU3_MASK                                     (0x00FE0000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU3_SHIFT                                    (0x00000011U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU3_MAX                                      (0x0000007FU)

#define CSL_BSS_DFE_TEST_SRC_CTRL_NOISE_SCALE_MASK                             (0x07000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NOISE_SCALE_SHIFT                            (0x00000018U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NOISE_SCALE_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NOISE_SCALE_MAX                              (0x00000007U)

#define CSL_BSS_DFE_TEST_SRC_CTRL_NU4_MASK                                     (0xF8000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU4_SHIFT                                    (0x0000001BU)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU4_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_TEST_SRC_CTRL_NU4_MAX                                      (0x0000001FU)

#define CSL_BSS_DFE_TEST_SRC_CTRL_RESETVAL                                     (0x00000000U)

/* BEAT_FREQ_TS1 */

#define CSL_BSS_DFE_BEAT_FREQ_TS1_BF_TS1_MASK                                  (0x0FFFFFFFU)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_BF_TS1_SHIFT                                 (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_BF_TS1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_BF_TS1_MAX                                   (0x0FFFFFFFU)

#define CSL_BSS_DFE_BEAT_FREQ_TS1_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS1_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_DFE_BEAT_FREQ_TS1_RESETVAL                                     (0x00000000U)

/* PHI_INIT_TS1 */

#define CSL_BSS_DFE_PHI_INIT_TS1_PHI_INIT_TS1_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_DFE_PHI_INIT_TS1_PHI_INIT_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS1_PHI_INIT_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS1_PHI_INIT_TS1_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_DFE_PHI_INIT_TS1_NU_MASK                                       (0xF0000000U)
#define CSL_BSS_DFE_PHI_INIT_TS1_NU_SHIFT                                      (0x0000001CU)
#define CSL_BSS_DFE_PHI_INIT_TS1_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS1_NU_MAX                                        (0x0000000FU)

#define CSL_BSS_DFE_PHI_INIT_TS1_RESETVAL                                      (0x00000000U)

/* BEAT_FREQ_TS2 */

#define CSL_BSS_DFE_BEAT_FREQ_TS2_BF_TS2_MASK                                  (0x0FFFFFFFU)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_BF_TS2_SHIFT                                 (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_BF_TS2_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_BF_TS2_MAX                                   (0x0FFFFFFFU)

#define CSL_BSS_DFE_BEAT_FREQ_TS2_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS2_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_DFE_BEAT_FREQ_TS2_RESETVAL                                     (0x00000000U)

/* PHI_INIT_TS2 */

#define CSL_BSS_DFE_PHI_INIT_TS2_PHI_INIT_TS2_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_DFE_PHI_INIT_TS2_PHI_INIT_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS2_PHI_INIT_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS2_PHI_INIT_TS2_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_DFE_PHI_INIT_TS2_NU_MASK                                       (0xF0000000U)
#define CSL_BSS_DFE_PHI_INIT_TS2_NU_SHIFT                                      (0x0000001CU)
#define CSL_BSS_DFE_PHI_INIT_TS2_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS2_NU_MAX                                        (0x0000000FU)

#define CSL_BSS_DFE_PHI_INIT_TS2_RESETVAL                                      (0x00000000U)

/* BEAT_FREQ_TS3 */

#define CSL_BSS_DFE_BEAT_FREQ_TS3_BF_TS3_MASK                                  (0x0FFFFFFFU)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_BF_TS3_SHIFT                                 (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_BF_TS3_RESETVAL                              (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_BF_TS3_MAX                                   (0x0FFFFFFFU)

#define CSL_BSS_DFE_BEAT_FREQ_TS3_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_BEAT_FREQ_TS3_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_DFE_BEAT_FREQ_TS3_RESETVAL                                     (0x00000000U)

/* PHI_INIT_TS3 */

#define CSL_BSS_DFE_PHI_INIT_TS3_PHI_INIT_TS3_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_DFE_PHI_INIT_TS3_PHI_INIT_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS3_PHI_INIT_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS3_PHI_INIT_TS3_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_DFE_PHI_INIT_TS3_NU_MASK                                       (0xF0000000U)
#define CSL_BSS_DFE_PHI_INIT_TS3_NU_SHIFT                                      (0x0000001CU)
#define CSL_BSS_DFE_PHI_INIT_TS3_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_PHI_INIT_TS3_NU_MAX                                        (0x0000000FU)

#define CSL_BSS_DFE_PHI_INIT_TS3_RESETVAL                                      (0x00000000U)

/* DF_PROFILE0_TS1 */

#define CSL_BSS_DFE_DF_PROFILE0_TS1_DF_P0_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_DF_P0_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_DF_P0_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_DF_P0_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE1_TS1 */

#define CSL_BSS_DFE_DF_PROFILE1_TS1_DF_P1_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_DF_P1_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_DF_P1_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_DF_P1_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE2_TS1 */

#define CSL_BSS_DFE_DF_PROFILE2_TS1_DF_P2_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_DF_P2_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_DF_P2_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_DF_P2_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE3_TS1 */

#define CSL_BSS_DFE_DF_PROFILE3_TS1_DF_P3_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_DF_P3_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_DF_P3_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_DF_P3_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE4_TS1 */

#define CSL_BSS_DFE_DF_PROFILE4_TS1_DF_P4_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_DF_P4_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_DF_P4_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_DF_P4_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE5_TS1 */

#define CSL_BSS_DFE_DF_PROFILE5_TS1_DF_P5_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_DF_P5_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_DF_P5_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_DF_P5_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE6_TS1 */

#define CSL_BSS_DFE_DF_PROFILE6_TS1_DF_P6_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_DF_P6_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_DF_P6_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_DF_P6_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE7_TS1 */

#define CSL_BSS_DFE_DF_PROFILE7_TS1_DF_P7_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_DF_P7_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_DF_P7_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_DF_P7_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE8_TS1 */

#define CSL_BSS_DFE_DF_PROFILE8_TS1_DF_P8_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_DF_P8_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_DF_P8_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_DF_P8_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE9_TS1 */

#define CSL_BSS_DFE_DF_PROFILE9_TS1_DF_P9_TS1_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_DF_P9_TS1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_DF_P9_TS1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_DF_P9_TS1_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS1_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS1_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS1_RESETVAL                                   (0x00000000U)

/* DF_PROFILE10_TS1 */

#define CSL_BSS_DFE_DF_PROFILE10_TS1_DF_P10_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_DF_P10_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_DF_P10_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_DF_P10_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE11_TS1 */

#define CSL_BSS_DFE_DF_PROFILE11_TS1_DF_P11_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_DF_P11_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_DF_P11_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_DF_P11_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE12_TS1 */

#define CSL_BSS_DFE_DF_PROFILE12_TS1_DF_P12_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_DF_P12_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_DF_P12_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_DF_P12_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE13_TS1 */

#define CSL_BSS_DFE_DF_PROFILE13_TS1_DF_P13_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_DF_P13_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_DF_P13_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_DF_P13_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE14_TS1 */

#define CSL_BSS_DFE_DF_PROFILE14_TS1_DF_P14_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_DF_P14_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_DF_P14_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_DF_P14_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE15_TS1 */

#define CSL_BSS_DFE_DF_PROFILE15_TS1_DF_P15_TS1_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_DF_P15_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_DF_P15_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_DF_P15_TS1_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS1_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS1_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS1_RESETVAL                                  (0x00000000U)

/* DF_PROFILE0_TS2 */

#define CSL_BSS_DFE_DF_PROFILE0_TS2_DF_P0_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_DF_P0_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_DF_P0_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_DF_P0_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE1_TS2 */

#define CSL_BSS_DFE_DF_PROFILE1_TS2_DF_P1_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_DF_P1_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_DF_P1_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_DF_P1_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE2_TS2 */

#define CSL_BSS_DFE_DF_PROFILE2_TS2_DF_P2_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_DF_P2_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_DF_P2_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_DF_P2_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE3_TS2 */

#define CSL_BSS_DFE_DF_PROFILE3_TS2_DF_P3_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_DF_P3_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_DF_P3_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_DF_P3_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE4_TS2 */

#define CSL_BSS_DFE_DF_PROFILE4_TS2_DF_P4_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_DF_P4_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_DF_P4_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_DF_P4_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE5_TS2 */

#define CSL_BSS_DFE_DF_PROFILE5_TS2_DF_P5_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_DF_P5_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_DF_P5_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_DF_P5_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE6_TS2 */

#define CSL_BSS_DFE_DF_PROFILE6_TS2_DF_P6_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_DF_P6_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_DF_P6_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_DF_P6_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE7_TS2 */

#define CSL_BSS_DFE_DF_PROFILE7_TS2_DF_P7_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_DF_P7_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_DF_P7_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_DF_P7_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE8_TS2 */

#define CSL_BSS_DFE_DF_PROFILE8_TS2_DF_P8_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_DF_P8_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_DF_P8_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_DF_P8_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE9_TS2 */

#define CSL_BSS_DFE_DF_PROFILE9_TS2_DF_P9_TS2_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_DF_P9_TS2_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_DF_P9_TS2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_DF_P9_TS2_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS2_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS2_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS2_RESETVAL                                   (0x00000000U)

/* DF_PROFILE10_TS2 */

#define CSL_BSS_DFE_DF_PROFILE10_TS2_DF_P10_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_DF_P10_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_DF_P10_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_DF_P10_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE11_TS2 */

#define CSL_BSS_DFE_DF_PROFILE11_TS2_DF_P11_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_DF_P11_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_DF_P11_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_DF_P11_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE12_TS2 */

#define CSL_BSS_DFE_DF_PROFILE12_TS2_DF_P12_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_DF_P12_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_DF_P12_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_DF_P12_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE13_TS2 */

#define CSL_BSS_DFE_DF_PROFILE13_TS2_DF_P13_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_DF_P13_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_DF_P13_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_DF_P13_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE14_TS2 */

#define CSL_BSS_DFE_DF_PROFILE14_TS2_DF_P14_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_DF_P14_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_DF_P14_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_DF_P14_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE15_TS2 */

#define CSL_BSS_DFE_DF_PROFILE15_TS2_DF_P15_TS2_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_DF_P15_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_DF_P15_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_DF_P15_TS2_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS2_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS2_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS2_RESETVAL                                  (0x00000000U)

/* DF_PROFILE0_TS3 */

#define CSL_BSS_DFE_DF_PROFILE0_TS3_DF_P0_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_DF_P0_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_DF_P0_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_DF_P0_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE0_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE0_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE1_TS3 */

#define CSL_BSS_DFE_DF_PROFILE1_TS3_DF_P1_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_DF_P1_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_DF_P1_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_DF_P1_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE1_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE1_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE2_TS3 */

#define CSL_BSS_DFE_DF_PROFILE2_TS3_DF_P2_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_DF_P2_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_DF_P2_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_DF_P2_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE2_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE2_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE3_TS3 */

#define CSL_BSS_DFE_DF_PROFILE3_TS3_DF_P3_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_DF_P3_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_DF_P3_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_DF_P3_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE3_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE3_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE4_TS3 */

#define CSL_BSS_DFE_DF_PROFILE4_TS3_DF_P4_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_DF_P4_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_DF_P4_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_DF_P4_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE4_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE4_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE5_TS3 */

#define CSL_BSS_DFE_DF_PROFILE5_TS3_DF_P5_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_DF_P5_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_DF_P5_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_DF_P5_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE5_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE5_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE6_TS3 */

#define CSL_BSS_DFE_DF_PROFILE6_TS3_DF_P6_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_DF_P6_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_DF_P6_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_DF_P6_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE6_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE6_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE7_TS3 */

#define CSL_BSS_DFE_DF_PROFILE7_TS3_DF_P7_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_DF_P7_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_DF_P7_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_DF_P7_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE7_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE7_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE8_TS3 */

#define CSL_BSS_DFE_DF_PROFILE8_TS3_DF_P8_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_DF_P8_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_DF_P8_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_DF_P8_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE8_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE8_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE9_TS3 */

#define CSL_BSS_DFE_DF_PROFILE9_TS3_DF_P9_TS3_MASK                             (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_DF_P9_TS3_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_DF_P9_TS3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_DF_P9_TS3_MAX                              (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS3_NU_MASK                                    (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_NU_SHIFT                                   (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_NU_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE9_TS3_NU_MAX                                     (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE9_TS3_RESETVAL                                   (0x00000000U)

/* DF_PROFILE10_TS3 */

#define CSL_BSS_DFE_DF_PROFILE10_TS3_DF_P10_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_DF_P10_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_DF_P10_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_DF_P10_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE10_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE10_TS3_RESETVAL                                  (0x00000000U)

/* DF_PROFILE11_TS3 */

#define CSL_BSS_DFE_DF_PROFILE11_TS3_DF_P11_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_DF_P11_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_DF_P11_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_DF_P11_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE11_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE11_TS3_RESETVAL                                  (0x00000000U)

/* DF_PROFILE12_TS3 */

#define CSL_BSS_DFE_DF_PROFILE12_TS3_DF_P12_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_DF_P12_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_DF_P12_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_DF_P12_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE12_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE12_TS3_RESETVAL                                  (0x00000000U)

/* DF_PROFILE13_TS3 */

#define CSL_BSS_DFE_DF_PROFILE13_TS3_DF_P13_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_DF_P13_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_DF_P13_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_DF_P13_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE13_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE13_TS3_RESETVAL                                  (0x00000000U)

/* DF_PROFILE14_TS3 */

#define CSL_BSS_DFE_DF_PROFILE14_TS3_DF_P14_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_DF_P14_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_DF_P14_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_DF_P14_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE14_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE14_TS3_RESETVAL                                  (0x00000000U)

/* DF_PROFILE15_TS3 */

#define CSL_BSS_DFE_DF_PROFILE15_TS3_DF_P15_TS3_MASK                           (0x003FFFFFU)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_DF_P15_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_DF_P15_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_DF_P15_TS3_MAX                            (0x003FFFFFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS3_NU_MASK                                   (0xFFC00000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_NU_SHIFT                                  (0x00000016U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_NU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DF_PROFILE15_TS3_NU_MAX                                    (0x000003FFU)

#define CSL_BSS_DFE_DF_PROFILE15_TS3_RESETVAL                                  (0x00000000U)

/* AMPLITUDE_SCALE_TS */

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS1_MASK                  (0x0000000FU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS1_SHIFT                 (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS1_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS1_MAX                   (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU1_MASK                                (0x000000F0U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU1_SHIFT                               (0x00000004U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU1_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU1_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS1_MASK                 (0x00007F00U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS1_SHIFT                (0x00000008U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS1_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS1_MAX                  (0x0000007FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU2_MASK                                (0x00008000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU2_SHIFT                               (0x0000000FU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU2_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU2_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS2_MASK                  (0x000F0000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS2_SHIFT                 (0x00000010U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS2_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_INT_AMP_SCALE_TS2_MAX                   (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU3_MASK                                (0x00F00000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU3_SHIFT                               (0x00000014U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU3_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU3_MAX                                 (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS2_MASK                 (0x7F000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS2_SHIFT                (0x00000018U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS2_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_FRAC_AMP_SCALE_TS2_MAX                  (0x0000007FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU4_MASK                                (0x80000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU4_SHIFT                               (0x0000001FU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU4_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_NU4_MAX                                 (0x00000001U)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS_RESETVAL                                (0x00000000U)

/* AMPLITUDE_SCALE_TS3 */

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_INT_AMP_SCALE_TS3_MASK                 (0x0000000FU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_INT_AMP_SCALE_TS3_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_INT_AMP_SCALE_TS3_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_INT_AMP_SCALE_TS3_MAX                  (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU1_MASK                               (0x000000F0U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU1_SHIFT                              (0x00000004U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU1_MAX                                (0x0000000FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_FRAC_AMP_SCALE_TS3_MASK                (0x00007F00U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_FRAC_AMP_SCALE_TS3_SHIFT               (0x00000008U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_FRAC_AMP_SCALE_TS3_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_FRAC_AMP_SCALE_TS3_MAX                 (0x0000007FU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU2_MASK                               (0xFFFF8000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU2_SHIFT                              (0x0000000FU)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_NU2_MAX                                (0x0001FFFFU)

#define CSL_BSS_DFE_AMPLITUDE_SCALE_TS3_RESETVAL                               (0x00000000U)

/* DELTA_FREQ_WORD_TS1 */

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_DELTA_FW_TS1_MASK                      (0x0001FFFFU)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_DELTA_FW_TS1_SHIFT                     (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_DELTA_FW_TS1_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_DELTA_FW_TS1_MAX                       (0x0001FFFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_NU_MASK                                (0xFFFE0000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_NU_SHIFT                               (0x00000011U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_NU_MAX                                 (0x00007FFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS1_RESETVAL                               (0x00000000U)

/* DELTA_FREQ_WORD_TS2 */

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_DELTA_FW_TS2_MASK                      (0x0001FFFFU)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_DELTA_FW_TS2_SHIFT                     (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_DELTA_FW_TS2_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_DELTA_FW_TS2_MAX                       (0x0001FFFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_NU_MASK                                (0xFFFE0000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_NU_SHIFT                               (0x00000011U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_NU_MAX                                 (0x00007FFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS2_RESETVAL                               (0x00000000U)

/* DELTA_FREQ_WORD_TS3 */

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_DELTA_FW_TS3_MASK                      (0x0001FFFFU)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_DELTA_FW_TS3_SHIFT                     (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_DELTA_FW_TS3_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_DELTA_FW_TS3_MAX                       (0x0001FFFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_NU_MASK                                (0xFFFE0000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_NU_SHIFT                               (0x00000011U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_NU_MAX                                 (0x00007FFFU)

#define CSL_BSS_DFE_DELTA_FREQ_WORD_TS3_RESETVAL                               (0x00000000U)

/* FREQ_SEED_TS1 */

#define CSL_BSS_DFE_FREQ_SEED_TS1_FREQ_SEED_TS1_MASK                           (0x01FFFFFFU)
#define CSL_BSS_DFE_FREQ_SEED_TS1_FREQ_SEED_TS1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS1_FREQ_SEED_TS1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS1_FREQ_SEED_TS1_MAX                            (0x01FFFFFFU)

#define CSL_BSS_DFE_FREQ_SEED_TS1_NU_MASK                                      (0xFE000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS1_NU_SHIFT                                     (0x00000019U)
#define CSL_BSS_DFE_FREQ_SEED_TS1_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS1_NU_MAX                                       (0x0000007FU)

#define CSL_BSS_DFE_FREQ_SEED_TS1_RESETVAL                                     (0x00000000U)

/* FREQ_SEED_TS2 */

#define CSL_BSS_DFE_FREQ_SEED_TS2_FREQ_SEED_TS2_MASK                           (0x01FFFFFFU)
#define CSL_BSS_DFE_FREQ_SEED_TS2_FREQ_SEED_TS2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS2_FREQ_SEED_TS2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS2_FREQ_SEED_TS2_MAX                            (0x01FFFFFFU)

#define CSL_BSS_DFE_FREQ_SEED_TS2_NU_MASK                                      (0xFE000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS2_NU_SHIFT                                     (0x00000019U)
#define CSL_BSS_DFE_FREQ_SEED_TS2_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS2_NU_MAX                                       (0x0000007FU)

#define CSL_BSS_DFE_FREQ_SEED_TS2_RESETVAL                                     (0x00000000U)

/* FREQ_SEED_TS3 */

#define CSL_BSS_DFE_FREQ_SEED_TS3_FREQ_SEED_TS3_MASK                           (0x01FFFFFFU)
#define CSL_BSS_DFE_FREQ_SEED_TS3_FREQ_SEED_TS3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS3_FREQ_SEED_TS3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS3_FREQ_SEED_TS3_MAX                            (0x01FFFFFFU)

#define CSL_BSS_DFE_FREQ_SEED_TS3_NU_MASK                                      (0xFE000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS3_NU_SHIFT                                     (0x00000019U)
#define CSL_BSS_DFE_FREQ_SEED_TS3_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_FREQ_SEED_TS3_NU_MAX                                       (0x0000007FU)

#define CSL_BSS_DFE_FREQ_SEED_TS3_RESETVAL                                     (0x00000000U)

/* MAX_FREQ_THRESHOLD */

#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_MAX_FREQ_TH_MASK                        (0x0001FFFFU)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_MAX_FREQ_TH_SHIFT                       (0x00000000U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_MAX_FREQ_TH_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_MAX_FREQ_TH_MAX                         (0x0001FFFFU)

#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_NU_MASK                                 (0xFFFE0000U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_NU_SHIFT                                (0x00000011U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_NU_RESETVAL                             (0x00000000U)
#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_NU_MAX                                  (0x00007FFFU)

#define CSL_BSS_DFE_MAX_FREQ_THRESHOLD_RESETVAL                                (0x00000000U)

/* MIN_FREQ_THRESHOLD */

#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_MIN_FREQ_TH_MASK                        (0x0001FFFFU)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_MIN_FREQ_TH_SHIFT                       (0x00000000U)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_MIN_FREQ_TH_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_MIN_FREQ_TH_MAX                         (0x0001FFFFU)

#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_NU_MASK                                 (0xFFFE0000U)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_NU_SHIFT                                (0x00000011U)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_NU_RESETVAL                             (0x00000000U)
#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_NU_MAX                                  (0x00007FFFU)

#define CSL_BSS_DFE_MIN_FREQ_THRESHOLD_RESETVAL                                (0x00000000U)

/* DOA_ROT_TS1_RX1 */

#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_I_TS1_RX1_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_I_TS1_RX1_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_I_TS1_RX1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_I_TS1_RX1_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_Q_TS1_RX1_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_Q_TS1_RX1_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_Q_TS1_RX1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_DOA_ROT_Q_TS1_RX1_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX1_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS2_RX1 */

#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_I_TS2_RX1_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_I_TS2_RX1_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_I_TS2_RX1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_I_TS2_RX1_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_Q_TS2_RX1_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_Q_TS2_RX1_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_Q_TS2_RX1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_DOA_ROT_Q_TS2_RX1_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX1_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS1_RX2 */

#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_I_TS1_RX2_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_I_TS1_RX2_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_I_TS1_RX2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_I_TS1_RX2_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_Q_TS1_RX2_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_Q_TS1_RX2_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_Q_TS1_RX2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_DOA_ROT_Q_TS1_RX2_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX2_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS2_RX2 */

#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_I_TS2_RX2_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_I_TS2_RX2_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_I_TS2_RX2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_I_TS2_RX2_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_Q_TS2_RX2_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_Q_TS2_RX2_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_Q_TS2_RX2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_DOA_ROT_Q_TS2_RX2_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX2_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS1_RX3 */

#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_I_TS1_RX3_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_I_TS1_RX3_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_I_TS1_RX3_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_I_TS1_RX3_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_Q_TS1_RX3_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_Q_TS1_RX3_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_Q_TS1_RX3_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_DOA_ROT_Q_TS1_RX3_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX3_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS2_RX3 */

#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_I_TS2_RX3_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_I_TS2_RX3_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_I_TS2_RX3_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_I_TS2_RX3_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_Q_TS2_RX3_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_Q_TS2_RX3_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_Q_TS2_RX3_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_DOA_ROT_Q_TS2_RX3_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX3_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS1_RX4 */

#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_I_TS1_RX4_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_I_TS1_RX4_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_I_TS1_RX4_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_I_TS1_RX4_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_Q_TS1_RX4_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_Q_TS1_RX4_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_Q_TS1_RX4_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_DOA_ROT_Q_TS1_RX4_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS1_RX4_RESETVAL                                   (0x00000000U)

/* DOA_ROT_TS2_RX4 */

#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_I_TS2_RX4_MASK                     (0x00000FFFU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_I_TS2_RX4_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_I_TS2_RX4_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_I_TS2_RX4_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU1_MASK                                   (0x0000F000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU1_SHIFT                                  (0x0000000CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU1_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_Q_TS2_RX4_MASK                     (0x0FFF0000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_Q_TS2_RX4_SHIFT                    (0x00000010U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_Q_TS2_RX4_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_DOA_ROT_Q_TS2_RX4_MAX                      (0x00000FFFU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU2_MASK                                   (0xF0000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU2_SHIFT                                  (0x0000001CU)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_NU2_MAX                                    (0x0000000FU)

#define CSL_BSS_DFE_DOA_ROT_TS2_RX4_RESETVAL                                   (0x00000000U)

/* TX_DFE_CTRL */

#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF1_MASK                            (0x00000001U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF1_SHIFT                           (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF1_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF1_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF2_MASK                            (0x00000002U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF2_SHIFT                           (0x00000001U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF2_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF2_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF3_MASK                            (0x00000004U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF3_SHIFT                           (0x00000002U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF3_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_PPD_ENABLE_RF3_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF1_MASK                            (0x00000008U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF1_SHIFT                           (0x00000003U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF1_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF1_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF2_MASK                            (0x00000010U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF2_SHIFT                           (0x00000004U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF2_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF2_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF3_MASK                            (0x00000020U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF3_SHIFT                           (0x00000005U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF3_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_COMP_EN_RF3_MAX                             (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_INIT_LUT_IDX_MASK                       (0x000000C0U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_INIT_LUT_IDX_SHIFT                      (0x00000006U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_INIT_LUT_IDX_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_INIT_LUT_IDX_MAX                        (0x00000003U)

#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_MEM_ACCESS_DFE_MASK                     (0x00000100U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_MEM_ACCESS_DFE_SHIFT                    (0x00000008U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_MEM_ACCESS_DFE_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_TX_PPD_MEM_ACCESS_DFE_MAX                      (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_NU1_MASK                                       (0x0000FE00U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU1_SHIFT                                      (0x00000009U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU1_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU1_MAX                                        (0x0000007FU)

#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF1_MASK                             (0x00030000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF1_SHIFT                            (0x00000010U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF1_MAX                              (0x00000003U)

#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF2_MASK                             (0x000C0000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF2_SHIFT                            (0x00000012U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF2_MAX                              (0x00000003U)

#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF3_MASK                             (0x00300000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF3_SHIFT                            (0x00000014U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_ORDER_RF3_MAX                              (0x00000003U)

#define CSL_BSS_DFE_TX_DFE_CTRL_NU2_MASK                                       (0x00C00000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU2_SHIFT                                      (0x00000016U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU2_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU2_MAX                                        (0x00000003U)

#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_EN_MASK                             (0x01000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_EN_SHIFT                            (0x00000018U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_EN_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_EN_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_NU3_MASK                                       (0x0E000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU3_SHIFT                                      (0x00000019U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU3_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU3_MAX                                        (0x00000007U)

#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_LEVEL_MASK                          (0x70000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_LEVEL_SHIFT                         (0x0000001CU)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_LEVEL_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_SDM_DITHER_LEVEL_MAX                           (0x00000007U)

#define CSL_BSS_DFE_TX_DFE_CTRL_NU4_MASK                                       (0x80000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU4_SHIFT                                      (0x0000001FU)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU4_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_CTRL_NU4_MAX                                        (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_CTRL_RESETVAL                                       (0x00000000U)

/* TX_BYPASS_CTRL */

#define CSL_BSS_DFE_TX_BYPASS_CTRL_ENABLE_ARBITRARY_DATA_TX_MASK               (0x00000001U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_ENABLE_ARBITRARY_DATA_TX_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_ENABLE_ARBITRARY_DATA_TX_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_ENABLE_ARBITRARY_DATA_TX_MAX                (0x00000001U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU1_MASK                                    (0x0000000EU)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU1_SHIFT                                   (0x00000001U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU1_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU1_MAX                                     (0x00000007U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_TX_DAC_MEM_ACCESS_DFE_MASK                  (0x00000010U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_TX_DAC_MEM_ACCESS_DFE_SHIFT                 (0x00000004U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_TX_DAC_MEM_ACCESS_DFE_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_TX_DAC_MEM_ACCESS_DFE_MAX                   (0x00000001U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU2_MASK                                    (0x000000E0U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU2_SHIFT                                   (0x00000005U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU2_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU2_MAX                                     (0x00000007U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_CFG_NUM_SAMPLES_ARB_TX_MASK                 (0x0003FF00U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_CFG_NUM_SAMPLES_ARB_TX_SHIFT                (0x00000008U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_CFG_NUM_SAMPLES_ARB_TX_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_CFG_NUM_SAMPLES_ARB_TX_MAX                  (0x000003FFU)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU3_MASK                                    (0x00FC0000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU3_SHIFT                                   (0x00000012U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU3_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU3_MAX                                     (0x0000003FU)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF1_MASK                  (0x01000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF1_SHIFT                 (0x00000018U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF1_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF1_MAX                   (0x00000001U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF2_MASK                  (0x02000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF2_SHIFT                 (0x00000019U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF2_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF2_MAX                   (0x00000001U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF3_MASK                  (0x04000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF3_SHIFT                 (0x0000001AU)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF3_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_BYPASS_DFE_OUTPUT_RF3_MAX                   (0x00000001U)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU4_MASK                                    (0xF8000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU4_SHIFT                                   (0x0000001BU)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU4_RESETVAL                                (0x00000000U)
#define CSL_BSS_DFE_TX_BYPASS_CTRL_NU4_MAX                                     (0x0000001FU)

#define CSL_BSS_DFE_TX_BYPASS_CTRL_RESETVAL                                    (0x00000000U)

/* TX_PHASE_SHIFT_BYPASS_VAL_RF1 */

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_I_RF1_MASK (0x000003FFU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_I_RF1_SHIFT (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_I_RF1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_I_RF1_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU1_MASK                     (0x0000FC00U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU1_SHIFT                    (0x0000000AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU1_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_Q_RF1_MASK (0x03FF0000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_Q_RF1_SHIFT (0x00000010U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_Q_RF1_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_PHASE_SHIFT_BYPASS_VAL_Q_RF1_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU2_MASK                     (0xFC000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU2_SHIFT                    (0x0000001AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_NU2_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF1_RESETVAL                     (0x00000000U)

/* TX_PHASE_SHIFT_BYPASS_VAL_RF2 */

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_I_RF2_MASK (0x000003FFU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_I_RF2_SHIFT (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_I_RF2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_I_RF2_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU1_MASK                     (0x0000FC00U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU1_SHIFT                    (0x0000000AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU1_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_Q_RF2_MASK (0x03FF0000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_Q_RF2_SHIFT (0x00000010U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_Q_RF2_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_PHASE_SHIFT_BYPASS_VAL_Q_RF2_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU2_MASK                     (0xFC000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU2_SHIFT                    (0x0000001AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_NU2_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF2_RESETVAL                     (0x00000000U)

/* TX_PHASE_SHIFT_BYPASS_VAL_RF3 */

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_I_RF3_MASK (0x000003FFU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_I_RF3_SHIFT (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_I_RF3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_I_RF3_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU1_MASK                     (0x0000FC00U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU1_SHIFT                    (0x0000000AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU1_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_Q_RF3_MASK (0x03FF0000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_Q_RF3_SHIFT (0x00000010U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_Q_RF3_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_PHASE_SHIFT_BYPASS_VAL_Q_RF3_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU2_MASK                     (0xFC000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU2_SHIFT                    (0x0000001AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_NU2_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF3_RESETVAL                     (0x00000000U)

/* TX_IQMM_MUX_SEL */

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF1_MASK                             (0x00000001U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF1_SHIFT                            (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF1_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF1_MASK                             (0x00000002U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF1_SHIFT                            (0x00000001U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF1_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF1_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU1_MASK                                   (0x0000000CU)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU1_SHIFT                                  (0x00000002U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU1_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF1_MASK                           (0x00000030U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF1_SHIFT                          (0x00000004U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF1_MASK                           (0x000000C0U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF1_SHIFT                          (0x00000006U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF1_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF2_MASK                             (0x00000100U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF2_SHIFT                            (0x00000008U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF2_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF2_MASK                             (0x00000200U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF2_SHIFT                            (0x00000009U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF2_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF2_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU2_MASK                                   (0x00000C00U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU2_SHIFT                                  (0x0000000AU)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU2_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF2_MASK                           (0x00003000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF2_SHIFT                          (0x0000000CU)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF2_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF2_MASK                           (0x0000C000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF2_SHIFT                          (0x0000000EU)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF2_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF3_MASK                             (0x00010000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF3_SHIFT                            (0x00000010U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SEL_RF3_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF3_MASK                             (0x00020000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF3_SHIFT                            (0x00000011U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF3_RESETVAL                         (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SEL_RF3_MAX                              (0x00000001U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU3_MASK                                   (0x000C0000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU3_SHIFT                                  (0x00000012U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU3_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU3_MAX                                    (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF3_MASK                           (0x00300000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF3_SHIFT                          (0x00000014U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_I_SCALE_RF3_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF3_MASK                           (0x00C00000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF3_SHIFT                          (0x00000016U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_Q_SCALE_RF3_MAX                            (0x00000003U)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU4_MASK                                   (0xFF000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU4_SHIFT                                  (0x00000018U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU4_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_NU4_MAX                                    (0x000000FFU)

#define CSL_BSS_DFE_TX_IQMM_MUX_SEL_RESETVAL                                   (0x00000000U)

/* TX_IQMM_GAIN_PH_CORR_RF1 */

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_GAIN_CORR_RF1_MASK                (0x00000FFFU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_GAIN_CORR_RF1_SHIFT               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_GAIN_CORR_RF1_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_GAIN_CORR_RF1_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU1_MASK                          (0x0000F000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU1_SHIFT                         (0x0000000CU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU1_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU1_MAX                           (0x0000000FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_PH_CORR_RF1_MASK                  (0x07FF0000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_PH_CORR_RF1_SHIFT                 (0x00000010U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_PH_CORR_RF1_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_PH_CORR_RF1_MAX                   (0x000007FFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU2_MASK                          (0xF8000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU2_SHIFT                         (0x0000001BU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU2_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_NU2_MAX                           (0x0000001FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF1_RESETVAL                          (0x00000000U)

/* TX_IQMM_GAIN_PH_CORR_RF2 */

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_GAIN_CORR_RF2_MASK                (0x00000FFFU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_GAIN_CORR_RF2_SHIFT               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_GAIN_CORR_RF2_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_GAIN_CORR_RF2_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU1_MASK                          (0x0000F000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU1_SHIFT                         (0x0000000CU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU1_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU1_MAX                           (0x0000000FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_PH_CORR_RF2_MASK                  (0x07FF0000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_PH_CORR_RF2_SHIFT                 (0x00000010U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_PH_CORR_RF2_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_PH_CORR_RF2_MAX                   (0x000007FFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU2_MASK                          (0xF8000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU2_SHIFT                         (0x0000001BU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU2_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_NU2_MAX                           (0x0000001FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF2_RESETVAL                          (0x00000000U)

/* TX_IQMM_GAIN_PH_CORR_RF3 */

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_GAIN_CORR_RF3_MASK                (0x00000FFFU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_GAIN_CORR_RF3_SHIFT               (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_GAIN_CORR_RF3_RESETVAL            (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_GAIN_CORR_RF3_MAX                 (0x00000FFFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU1_MASK                          (0x0000F000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU1_SHIFT                         (0x0000000CU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU1_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU1_MAX                           (0x0000000FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_PH_CORR_RF3_MASK                  (0x07FF0000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_PH_CORR_RF3_SHIFT                 (0x00000010U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_PH_CORR_RF3_RESETVAL              (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_PH_CORR_RF3_MAX                   (0x000007FFU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU2_MASK                          (0xF8000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU2_SHIFT                         (0x0000001BU)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU2_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_NU2_MAX                           (0x0000001FU)

#define CSL_BSS_DFE_TX_IQMM_GAIN_PH_CORR_RF3_RESETVAL                          (0x00000000U)

/* TX_RF_MM_EQ_TAU */

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_TAU_MASK                                   (0x0000FFFFU)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_TAU_SHIFT                                  (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_TAU_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_TAU_MAX                                    (0x0000FFFFU)

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_MEM_ACCESS_INTERNAL_MASK                   (0x00010000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_MEM_ACCESS_INTERNAL_SHIFT                  (0x00000010U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_MEM_ACCESS_INTERNAL_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_MEM_ACCESS_INTERNAL_MAX                    (0x00000001U)

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_NU2_MASK                                   (0xFFFE0000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_NU2_SHIFT                                  (0x00000011U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_NU2_MAX                                    (0x00007FFFU)

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_RESETVAL                                   (0x00000000U)

/* TX_RF_MM_EQ_TAU_INIT_VAL */

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_TAU_INIT_VAL_MASK                 (0x007FFFFFU)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_TAU_INIT_VAL_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_TAU_INIT_VAL_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_TAU_INIT_VAL_MAX                  (0x007FFFFFU)

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_NU_MASK                           (0xFF800000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_NU_SHIFT                          (0x00000017U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_NU_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_NU_MAX                            (0x000001FFU)

#define CSL_BSS_DFE_TX_RF_MM_EQ_TAU_INIT_VAL_RESETVAL                          (0x00000000U)

/* TX_PPD_MUVAL */

#define CSL_BSS_DFE_TX_PPD_MUVAL_INIT_VAL_MUACC_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_TX_PPD_MUVAL_INIT_VAL_MUACC_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUVAL_INIT_VAL_MUACC_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUVAL_INIT_VAL_MUACC_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_TX_PPD_MUVAL_RESETVAL                                      (0x00000000U)

/* TX_PPD_MUACC1 */

#define CSL_BSS_DFE_TX_PPD_MUACC1_SLOPEOFMUACC1_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_TX_PPD_MUACC1_SLOPEOFMUACC1_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC1_SLOPEOFMUACC1_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC1_SLOPEOFMUACC1_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_TX_PPD_MUACC1_RESETVAL                                     (0x00000000U)

/* TX_PPD_MUACC2 */

#define CSL_BSS_DFE_TX_PPD_MUACC2_SLOPEOFMUACC2_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_TX_PPD_MUACC2_SLOPEOFMUACC2_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC2_SLOPEOFMUACC2_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC2_SLOPEOFMUACC2_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_TX_PPD_MUACC2_RESETVAL                                     (0x00000000U)

/* TX_PPD_MUACC3 */

#define CSL_BSS_DFE_TX_PPD_MUACC3_SLOPEOFMUACC3_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_TX_PPD_MUACC3_SLOPEOFMUACC3_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC3_SLOPEOFMUACC3_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_TX_PPD_MUACC3_SLOPEOFMUACC3_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_TX_PPD_MUACC3_RESETVAL                                     (0x00000000U)

/* MUX_TXD_RF_CFG */

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE0_MASK              (0x0000000FU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE0_SHIFT             (0x00000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE0_RESETVAL          (0x00000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE0_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE1_MASK              (0x000000F0U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE1_SHIFT             (0x00000004U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE1_RESETVAL          (0x00000001U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE1_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE2_MASK              (0x00000F00U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE2_SHIFT             (0x00000008U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE2_RESETVAL          (0x00000002U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE2_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE3_MASK              (0x0000F000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE3_SHIFT             (0x0000000CU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE3_RESETVAL          (0x00000009U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE3_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE4_MASK              (0x000F0000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE4_SHIFT             (0x00000010U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE4_RESETVAL          (0x0000000CU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE4_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE5_MASK              (0x00F00000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE5_SHIFT             (0x00000014U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE5_RESETVAL          (0x0000000DU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE5_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE6_MASK              (0x0F000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE6_SHIFT             (0x00000018U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE6_RESETVAL          (0x0000000EU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE6_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE7_MASK              (0xF0000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE7_SHIFT             (0x0000001CU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE7_RESETVAL          (0x00000009U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_MUX_TXD_RF_CFG_TX_ENABLE7_MAX               (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_RESETVAL                                    (0x9EDC9210U)

/* TX_DC_CORR_RF1 */

#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_I_RF1_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_I_RF1_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_I_RF1_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_I_RF1_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_Q_RF1_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_Q_RF1_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_Q_RF1_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF1_TX_DC_CORR_Q_RF1_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF1_RESETVAL                                    (0x00000000U)

/* TX_DC_CORR_RF2 */

#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_I_RF2_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_I_RF2_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_I_RF2_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_I_RF2_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_Q_RF2_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_Q_RF2_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_Q_RF2_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF2_TX_DC_CORR_Q_RF2_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF2_RESETVAL                                    (0x00000000U)

/* TX_DC_CORR_RF3 */

#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_I_RF3_MASK                       (0x0000FFFFU)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_I_RF3_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_I_RF3_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_I_RF3_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_Q_RF3_MASK                       (0xFFFF0000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_Q_RF3_SHIFT                      (0x00000010U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_Q_RF3_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_TX_DC_CORR_RF3_TX_DC_CORR_Q_RF3_MAX                        (0x0000FFFFU)

#define CSL_BSS_DFE_TX_DC_CORR_RF3_RESETVAL                                    (0x00000000U)

/* RX_IQMM_CFG1 */

#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_ACCUM1_INIT_MASK                       (0x001FFFFFU)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_ACCUM1_INIT_SHIFT                      (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_ACCUM1_INIT_RESETVAL                   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_ACCUM1_INIT_MAX                        (0x001FFFFFU)

#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_SLOPE_SIGN_MASK                        (0x00600000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_SLOPE_SIGN_SHIFT                       (0x00000015U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_SLOPE_SIGN_RESETVAL                    (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_SLOPE_SIGN_MAX                         (0x00000003U)

#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_INIT_LUT_INDEX_MASK                    (0x0F800000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_INIT_LUT_INDEX_SHIFT                   (0x00000017U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_INIT_LUT_INDEX_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_RXIQMM_INIT_LUT_INDEX_MAX                     (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CFG1_NU_MASK                                       (0xF0000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_NU_SHIFT                                      (0x0000001CU)
#define CSL_BSS_DFE_RX_IQMM_CFG1_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG1_NU_MAX                                        (0x0000000FU)

#define CSL_BSS_DFE_RX_IQMM_CFG1_RESETVAL                                      (0x00000000U)

/* RX_IQMM_CFG2 */

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_FREQ_UPDATE_VAL_MASK                   (0x00003FFFU)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_FREQ_UPDATE_VAL_SHIFT                  (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_FREQ_UPDATE_VAL_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_FREQ_UPDATE_VAL_MAX                    (0x00003FFFU)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_GAIN_CORR_STATIC_MODE_EN_MASK          (0x00004000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_GAIN_CORR_STATIC_MODE_EN_SHIFT         (0x0000000EU)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_GAIN_CORR_STATIC_MODE_EN_RESETVAL      (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_GAIN_CORR_STATIC_MODE_EN_MAX           (0x00000001U)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_PHASE_CORR_STATIC_MODE_EN_MASK         (0x00008000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_PHASE_CORR_STATIC_MODE_EN_SHIFT        (0x0000000FU)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_PHASE_CORR_STATIC_MODE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_PHASE_CORR_STATIC_MODE_EN_MAX          (0x00000001U)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_EN_MASK                         (0x00010000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_EN_SHIFT                        (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_EN_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_EN_MAX                          (0x00000001U)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC1_MASK            (0x003E0000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC1_SHIFT           (0x00000011U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC1_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC1_MAX             (0x0000001FU)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC2_MASK            (0x03C00000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC2_SHIFT           (0x00000016U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC2_RESETVAL        (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_RXIQMM_DITHER_RED_FACTOR_ACC2_MAX             (0x0000000FU)

#define CSL_BSS_DFE_RX_IQMM_CFG2_NU_MASK                                       (0xFC000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_NU_SHIFT                                      (0x0000001AU)
#define CSL_BSS_DFE_RX_IQMM_CFG2_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG2_NU_MAX                                        (0x0000003FU)

#define CSL_BSS_DFE_RX_IQMM_CFG2_RESETVAL                                      (0x00000000U)

/* RX_IQMM_CFG3 */

#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ACCUM1_THRES_MASK                      (0x001FFFFFU)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ACCUM1_THRES_SHIFT                     (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ACCUM1_THRES_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ACCUM1_THRES_MAX                       (0x001FFFFFU)

#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_MEM_ACCESS_DFE_MASK                    (0x00200000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_MEM_ACCESS_DFE_SHIFT                   (0x00000015U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_MEM_ACCESS_DFE_RESETVAL                (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_MEM_ACCESS_DFE_MAX                     (0x00000001U)

#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ENABLE_MASK                            (0x00400000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ENABLE_SHIFT                           (0x00000016U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_RXIQMM_ENABLE_MAX                             (0x00000001U)

#define CSL_BSS_DFE_RX_IQMM_CFG3_NU_MASK                                       (0xFF800000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_NU_SHIFT                                      (0x00000017U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG3_NU_MAX                                        (0x000001FFU)

#define CSL_BSS_DFE_RX_IQMM_CFG3_RESETVAL                                      (0x00000000U)

/* RX_IQMM_CFG4 */

#define CSL_BSS_DFE_RX_IQMM_CFG4_RXIQMM_ACCUM2_THRES_MASK                      (0x0000FFFFU)
#define CSL_BSS_DFE_RX_IQMM_CFG4_RXIQMM_ACCUM2_THRES_SHIFT                     (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG4_RXIQMM_ACCUM2_THRES_RESETVAL                  (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG4_RXIQMM_ACCUM2_THRES_MAX                       (0x0000FFFFU)

#define CSL_BSS_DFE_RX_IQMM_CFG4_NU_MASK                                       (0xFFFF0000U)
#define CSL_BSS_DFE_RX_IQMM_CFG4_NU_SHIFT                                      (0x00000010U)
#define CSL_BSS_DFE_RX_IQMM_CFG4_NU_RESETVAL                                   (0x00000000U)
#define CSL_BSS_DFE_RX_IQMM_CFG4_NU_MAX                                        (0x0000FFFFU)

#define CSL_BSS_DFE_RX_IQMM_CFG4_RESETVAL                                      (0x00000000U)

/* CHIRP_MASK_CFG */

#define CSL_BSS_DFE_CHIRP_MASK_CFG_DSS_IF_MASK_MASK                            (0x00000001U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_DSS_IF_MASK_SHIFT                           (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_DSS_IF_MASK_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_DSS_IF_MASK_MAX                             (0x00000001U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_AGC_STATS_MASK_MASK                         (0x00000002U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_AGC_STATS_MASK_SHIFT                        (0x00000001U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_AGC_STATS_MASK_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_AGC_STATS_MASK_MAX                          (0x00000001U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_WBI_MASK_MASK                            (0x00000004U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_WBI_MASK_SHIFT                           (0x00000002U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_WBI_MASK_RESETVAL                        (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_WBI_MASK_MAX                             (0x00000001U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_SIGIMG_MASK_MASK                         (0x00000008U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_SIGIMG_MASK_SHIFT                        (0x00000003U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_SIGIMG_MASK_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_SIGIMG_MASK_MAX                          (0x00000001U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_ADCSAT_MASK_MASK                         (0x00000010U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_ADCSAT_MASK_SHIFT                        (0x00000004U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_ADCSAT_MASK_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_CQ_ADCSAT_MASK_MAX                          (0x00000001U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_MISC_MASK_MASK                              (0x000000E0U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_MISC_MASK_SHIFT                             (0x00000005U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_MISC_MASK_RESETVAL                          (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_MISC_MASK_MAX                               (0x00000007U)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_NU_MASK                                     (0xFFFFFF00U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_NU_SHIFT                                    (0x00000008U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DFE_CHIRP_MASK_CFG_NU_MAX                                      (0x00FFFFFFU)

#define CSL_BSS_DFE_CHIRP_MASK_CFG_RESETVAL                                    (0x00000000U)

/* CHIRP_COUNT_OVERRIDE_CFG */

#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_VAL_MASK     (0x00000FFFU)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_VAL_SHIFT    (0x00000000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_VAL_MAX      (0x00000FFFU)

#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU1_MASK                          (0x0000F000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU1_SHIFT                         (0x0000000CU)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU1_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU1_MAX                           (0x0000000FU)

#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_CONTROL_MASK (0x00010000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_CONTROL_SHIFT (0x00000010U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_CONTROL_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_CHIRP_COUNT_OVERRIDE_CONTROL_MAX  (0x00000001U)

#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU2_MASK                          (0xFFFE0000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU2_SHIFT                         (0x00000011U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU2_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_NU2_MAX                           (0x00007FFFU)

#define CSL_BSS_DFE_CHIRP_COUNT_OVERRIDE_CFG_RESETVAL                          (0x00000000U)

/* DFE_RESERVED_2 */

#define CSL_BSS_DFE_DFE_RESERVED_2_DFE_RESERVED_2_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_DFE_DFE_RESERVED_2_DFE_RESERVED_2_SHIFT                        (0x00000000U)
#define CSL_BSS_DFE_DFE_RESERVED_2_DFE_RESERVED_2_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_DFE_RESERVED_2_DFE_RESERVED_2_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_DFE_DFE_RESERVED_2_RESETVAL                                    (0x00000000U)

/* AGC_RX1_ICH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB_AGC_RX1_ICH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB_AGC_RX1_ICH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB_AGC_RX1_ICH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB_AGC_RX1_ICH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_ICH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB_AGC_RX1_ICH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB_AGC_RX1_ICH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB_AGC_RX1_ICH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB_AGC_RX1_ICH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_ICH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_QCH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB_AGC_RX1_QCH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB_AGC_RX1_QCH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB_AGC_RX1_QCH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB_AGC_RX1_QCH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_QCH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB_AGC_RX1_QCH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB_AGC_RX1_QCH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB_AGC_RX1_QCH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB_AGC_RX1_QCH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_QCH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_ISQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB_AGC_RX1_ISQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB_AGC_RX1_ISQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB_AGC_RX1_ISQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB_AGC_RX1_ISQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_ISQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB_AGC_RX1_ISQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB_AGC_RX1_ISQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB_AGC_RX1_ISQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB_AGC_RX1_ISQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_ISQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_QSQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB_AGC_RX1_QSQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB_AGC_RX1_QSQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB_AGC_RX1_QSQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB_AGC_RX1_QSQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_QSQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB_AGC_RX1_QSQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB_AGC_RX1_QSQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB_AGC_RX1_QSQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB_AGC_RX1_QSQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_QSQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX1_IQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB_AGC_RX1_IQ_ACC_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB_AGC_RX1_IQ_ACC_LSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB_AGC_RX1_IQ_ACC_LSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB_AGC_RX1_IQ_ACC_LSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_LSB_RESETVAL                                (0x00000000U)

/* AGC_RX1_IQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB_AGC_RX1_IQ_ACC_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB_AGC_RX1_IQ_ACC_MSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB_AGC_RX1_IQ_ACC_MSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB_AGC_RX1_IQ_ACC_MSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX1_IQ_ACC_MSB_RESETVAL                                (0x00000000U)

/* AGC_RX2_ICH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB_AGC_RX2_ICH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB_AGC_RX2_ICH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB_AGC_RX2_ICH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB_AGC_RX2_ICH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_ICH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB_AGC_RX2_ICH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB_AGC_RX2_ICH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB_AGC_RX2_ICH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB_AGC_RX2_ICH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_ICH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_QCH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB_AGC_RX2_QCH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB_AGC_RX2_QCH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB_AGC_RX2_QCH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB_AGC_RX2_QCH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_QCH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB_AGC_RX2_QCH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB_AGC_RX2_QCH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB_AGC_RX2_QCH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB_AGC_RX2_QCH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_QCH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_ISQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB_AGC_RX2_ISQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB_AGC_RX2_ISQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB_AGC_RX2_ISQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB_AGC_RX2_ISQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_ISQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB_AGC_RX2_ISQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB_AGC_RX2_ISQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB_AGC_RX2_ISQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB_AGC_RX2_ISQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_ISQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_QSQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB_AGC_RX2_QSQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB_AGC_RX2_QSQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB_AGC_RX2_QSQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB_AGC_RX2_QSQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_QSQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB_AGC_RX2_QSQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB_AGC_RX2_QSQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB_AGC_RX2_QSQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB_AGC_RX2_QSQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_QSQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX2_IQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB_AGC_RX2_IQ_ACC_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB_AGC_RX2_IQ_ACC_LSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB_AGC_RX2_IQ_ACC_LSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB_AGC_RX2_IQ_ACC_LSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_LSB_RESETVAL                                (0x00000000U)

/* AGC_RX2_IQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB_AGC_RX2_IQ_ACC_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB_AGC_RX2_IQ_ACC_MSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB_AGC_RX2_IQ_ACC_MSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB_AGC_RX2_IQ_ACC_MSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX2_IQ_ACC_MSB_RESETVAL                                (0x00000000U)

/* AGC_RX3_ICH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB_AGC_RX3_ICH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB_AGC_RX3_ICH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB_AGC_RX3_ICH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB_AGC_RX3_ICH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_ICH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB_AGC_RX3_ICH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB_AGC_RX3_ICH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB_AGC_RX3_ICH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB_AGC_RX3_ICH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_ICH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_QCH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB_AGC_RX3_QCH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB_AGC_RX3_QCH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB_AGC_RX3_QCH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB_AGC_RX3_QCH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_QCH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB_AGC_RX3_QCH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB_AGC_RX3_QCH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB_AGC_RX3_QCH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB_AGC_RX3_QCH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_QCH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_ISQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB_AGC_RX3_ISQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB_AGC_RX3_ISQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB_AGC_RX3_ISQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB_AGC_RX3_ISQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_ISQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB_AGC_RX3_ISQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB_AGC_RX3_ISQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB_AGC_RX3_ISQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB_AGC_RX3_ISQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_ISQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_QSQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB_AGC_RX3_QSQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB_AGC_RX3_QSQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB_AGC_RX3_QSQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB_AGC_RX3_QSQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_QSQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB_AGC_RX3_QSQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB_AGC_RX3_QSQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB_AGC_RX3_QSQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB_AGC_RX3_QSQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_QSQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX3_IQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB_AGC_RX3_IQ_ACC_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB_AGC_RX3_IQ_ACC_LSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB_AGC_RX3_IQ_ACC_LSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB_AGC_RX3_IQ_ACC_LSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_LSB_RESETVAL                                (0x00000000U)

/* AGC_RX3_IQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB_AGC_RX3_IQ_ACC_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB_AGC_RX3_IQ_ACC_MSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB_AGC_RX3_IQ_ACC_MSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB_AGC_RX3_IQ_ACC_MSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX3_IQ_ACC_MSB_RESETVAL                                (0x00000000U)

/* AGC_RX4_ICH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB_AGC_RX4_ICH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB_AGC_RX4_ICH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB_AGC_RX4_ICH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB_AGC_RX4_ICH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_ICH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB_AGC_RX4_ICH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB_AGC_RX4_ICH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB_AGC_RX4_ICH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB_AGC_RX4_ICH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_ICH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_QCH_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB_AGC_RX4_QCH_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB_AGC_RX4_QCH_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB_AGC_RX4_QCH_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB_AGC_RX4_QCH_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_QCH_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB_AGC_RX4_QCH_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB_AGC_RX4_QCH_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB_AGC_RX4_QCH_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB_AGC_RX4_QCH_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_QCH_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_ISQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB_AGC_RX4_ISQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB_AGC_RX4_ISQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB_AGC_RX4_ISQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB_AGC_RX4_ISQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_ISQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB_AGC_RX4_ISQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB_AGC_RX4_ISQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB_AGC_RX4_ISQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB_AGC_RX4_ISQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_ISQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_QSQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB_AGC_RX4_QSQ_ACC_LSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB_AGC_RX4_QSQ_ACC_LSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB_AGC_RX4_QSQ_ACC_LSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB_AGC_RX4_QSQ_ACC_LSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_LSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_QSQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB_AGC_RX4_QSQ_ACC_MSB_MASK               (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB_AGC_RX4_QSQ_ACC_MSB_SHIFT              (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB_AGC_RX4_QSQ_ACC_MSB_RESETVAL           (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB_AGC_RX4_QSQ_ACC_MSB_MAX                (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_QSQ_ACC_MSB_RESETVAL                               (0x00000000U)

/* AGC_RX4_IQ_ACC_LSB */

#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB_AGC_RX4_IQ_ACC_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB_AGC_RX4_IQ_ACC_LSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB_AGC_RX4_IQ_ACC_LSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB_AGC_RX4_IQ_ACC_LSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_LSB_RESETVAL                                (0x00000000U)

/* AGC_RX4_IQ_ACC_MSB */

#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB_AGC_RX4_IQ_ACC_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB_AGC_RX4_IQ_ACC_MSB_SHIFT                (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB_AGC_RX4_IQ_ACC_MSB_RESETVAL             (0x00000000U)
#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB_AGC_RX4_IQ_ACC_MSB_MAX                  (0xFFFFFFFFU)

#define CSL_BSS_DFE_AGC_RX4_IQ_ACC_MSB_RESETVAL                                (0x00000000U)

/* AGC_COUNT_ACC */

#define CSL_BSS_DFE_AGC_COUNT_ACC_AGC_COUNT_ACC_MASK                           (0x00FFFFFFU)
#define CSL_BSS_DFE_AGC_COUNT_ACC_AGC_COUNT_ACC_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_AGC_COUNT_ACC_AGC_COUNT_ACC_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_AGC_COUNT_ACC_AGC_COUNT_ACC_MAX                            (0x00FFFFFFU)

#define CSL_BSS_DFE_AGC_COUNT_ACC_NU_MASK                                      (0xFF000000U)
#define CSL_BSS_DFE_AGC_COUNT_ACC_NU_SHIFT                                     (0x00000018U)
#define CSL_BSS_DFE_AGC_COUNT_ACC_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DFE_AGC_COUNT_ACC_NU_MAX                                       (0x000000FFU)

#define CSL_BSS_DFE_AGC_COUNT_ACC_RESETVAL                                     (0x00000000U)

/* PARITY_OR_STATUS */

#define CSL_BSS_DFE_PARITY_OR_STATUS_PARITY_OR_STATUS_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_DFE_PARITY_OR_STATUS_PARITY_OR_STATUS_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_PARITY_OR_STATUS_PARITY_OR_STATUS_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_PARITY_OR_STATUS_PARITY_OR_STATUS_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_DFE_PARITY_OR_STATUS_RESETVAL                                  (0x00000000U)

/* PARITY_AND_STATUS */

#define CSL_BSS_DFE_PARITY_AND_STATUS_PARITY_AND_STATUS_MASK                   (0xFFFFFFFFU)
#define CSL_BSS_DFE_PARITY_AND_STATUS_PARITY_AND_STATUS_SHIFT                  (0x00000000U)
#define CSL_BSS_DFE_PARITY_AND_STATUS_PARITY_AND_STATUS_RESETVAL               (0x00000000U)
#define CSL_BSS_DFE_PARITY_AND_STATUS_PARITY_AND_STATUS_MAX                    (0xFFFFFFFFU)

#define CSL_BSS_DFE_PARITY_AND_STATUS_RESETVAL                                 (0x00000000U)

/* TX_DFE_SWAP_FLIP */

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF1_MASK                          (0x00000001U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF1_SHIFT                         (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF1_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF1_MAX                           (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF2_MASK                          (0x00000002U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF2_SHIFT                         (0x00000001U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF2_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF2_MAX                           (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF3_MASK                          (0x00000004U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF3_SHIFT                         (0x00000002U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF3_RESETVAL                      (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_IQ_SWAP_RF3_MAX                           (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_MSB_LSB_FLIP_MASK                         (0x00000008U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_MSB_LSB_FLIP_SHIFT                        (0x00000003U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_MSB_LSB_FLIP_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_MSB_LSB_FLIP_MAX                          (0x00000001U)

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_SPARE_TX_DFE_MASK                         (0xFFFFFFF0U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_SPARE_TX_DFE_SHIFT                        (0x00000004U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_SPARE_TX_DFE_RESETVAL                     (0x00000000U)
#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_SPARE_TX_DFE_MAX                          (0x0FFFFFFFU)

#define CSL_BSS_DFE_TX_DFE_SWAP_FLIP_RESETVAL                                  (0x00000000U)

/* CQ3_RX1_IQ_RD */

#define CSL_BSS_DFE_CQ3_RX1_IQ_RD_CQ3_RX1_IQ_RD_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_CQ3_RX1_IQ_RD_CQ3_RX1_IQ_RD_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX1_IQ_RD_CQ3_RX1_IQ_RD_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX1_IQ_RD_CQ3_RX1_IQ_RD_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_CQ3_RX1_IQ_RD_RESETVAL                                     (0x00000000U)

/* CQ3_RX2_IQ_RD */

#define CSL_BSS_DFE_CQ3_RX2_IQ_RD_CQ3_RX2_IQ_RD_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_CQ3_RX2_IQ_RD_CQ3_RX2_IQ_RD_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX2_IQ_RD_CQ3_RX2_IQ_RD_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX2_IQ_RD_CQ3_RX2_IQ_RD_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_CQ3_RX2_IQ_RD_RESETVAL                                     (0x00000000U)

/* CQ3_RX3_IQ_RD */

#define CSL_BSS_DFE_CQ3_RX3_IQ_RD_CQ3_RX3_IQ_RD_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_CQ3_RX3_IQ_RD_CQ3_RX3_IQ_RD_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX3_IQ_RD_CQ3_RX3_IQ_RD_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX3_IQ_RD_CQ3_RX3_IQ_RD_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_CQ3_RX3_IQ_RD_RESETVAL                                     (0x00000000U)

/* CQ3_RX4_IQ_RD */

#define CSL_BSS_DFE_CQ3_RX4_IQ_RD_CQ3_RX4_IQ_RD_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DFE_CQ3_RX4_IQ_RD_CQ3_RX4_IQ_RD_SHIFT                          (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX4_IQ_RD_CQ3_RX4_IQ_RD_RESETVAL                       (0x00000000U)
#define CSL_BSS_DFE_CQ3_RX4_IQ_RD_CQ3_RX4_IQ_RD_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DFE_CQ3_RX4_IQ_RD_RESETVAL                                     (0x00000000U)

/* MUX_TXD_RF_CFG_TX4 */

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE8_MASK          (0x0000000FU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE8_SHIFT         (0x00000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE8_RESETVAL      (0x00000001U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE8_MAX           (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE9_MASK          (0x000000F0U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE9_SHIFT         (0x00000004U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE9_RESETVAL      (0x00000001U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE9_MAX           (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE10_MASK         (0x00000F00U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE10_SHIFT        (0x00000008U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE10_RESETVAL     (0x00000002U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE10_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE11_MASK         (0x0000F000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE11_SHIFT        (0x0000000CU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE11_RESETVAL     (0x00000009U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE11_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE12_MASK         (0x000F0000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE12_SHIFT        (0x00000010U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE12_RESETVAL     (0x00000003U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE12_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE13_MASK         (0x00F00000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE13_SHIFT        (0x00000014U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE13_RESETVAL     (0x0000000DU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE13_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE14_MASK         (0x0F000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE14_SHIFT        (0x00000018U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE14_RESETVAL     (0x0000000EU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE14_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE15_MASK         (0xF0000000U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE15_SHIFT        (0x0000001CU)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE15_RESETVAL     (0x00000009U)
#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_MUX_TXD_RF_CFG_TX_ENABLE15_MAX          (0x0000000FU)

#define CSL_BSS_DFE_MUX_TXD_RF_CFG_TX4_RESETVAL                                (0x9ED39211U)

/* TX4_DFE_CFG */

#define CSL_BSS_DFE_TX4_DFE_CFG_BYPASS_DFE_OUTPUT_RF4_MASK                     (0x00000001U)
#define CSL_BSS_DFE_TX4_DFE_CFG_BYPASS_DFE_OUTPUT_RF4_SHIFT                    (0x00000000U)
#define CSL_BSS_DFE_TX4_DFE_CFG_BYPASS_DFE_OUTPUT_RF4_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX4_DFE_CFG_BYPASS_DFE_OUTPUT_RF4_MAX                      (0x00000001U)

#define CSL_BSS_DFE_TX4_DFE_CFG_NU_MASK                                        (0xFFFFFE00U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_SHIFT                                       (0x00000009U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_RESETVAL                                    (0x00000000U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_MAX                                         (0x007FFFFFU)

#define CSL_BSS_DFE_TX4_DFE_CFG_IQ_SWAP_RF4_MASK                               (0x00000100U)
#define CSL_BSS_DFE_TX4_DFE_CFG_IQ_SWAP_RF4_SHIFT                              (0x00000008U)
#define CSL_BSS_DFE_TX4_DFE_CFG_IQ_SWAP_RF4_RESETVAL                           (0x00000000U)
#define CSL_BSS_DFE_TX4_DFE_CFG_IQ_SWAP_RF4_MAX                                (0x00000001U)

#define CSL_BSS_DFE_TX4_DFE_CFG_NU_MASK                                        (0xFFFFFE00U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_SHIFT                                       (0x00000009U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_RESETVAL                                    (0x00000000U)
#define CSL_BSS_DFE_TX4_DFE_CFG_NU_MAX                                         (0x007FFFFFU)

#define CSL_BSS_DFE_TX4_DFE_CFG_RESETVAL                                       (0x00000000U)

/* TX_PHASE_SHIFT_BYPASS_VAL_RF4 */

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_I_RF4_MASK (0x000003FFU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_I_RF4_SHIFT (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_I_RF4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_I_RF4_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU1_MASK                     (0x0000FC00U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU1_SHIFT                    (0x0000000AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU1_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU1_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_Q_RF4_MASK (0x03FF0000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_Q_RF4_SHIFT (0x00000010U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_Q_RF4_RESETVAL (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_PHASE_SHIFT_BYPASS_VAL_Q_RF4_MAX (0x000003FFU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU2_MASK                     (0xFC000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU2_SHIFT                    (0x0000001AU)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU2_RESETVAL                 (0x00000000U)
#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_NU2_MAX                      (0x0000003FU)

#define CSL_BSS_DFE_TX_PHASE_SHIFT_BYPASS_VAL_RF4_RESETVAL                     (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
