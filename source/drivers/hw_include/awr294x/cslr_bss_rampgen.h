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
 *  Name        : cslr_bss_rampgen.h
*/
#ifndef CSLR_BSS_RAMPGEN_H_
#define CSLR_BSS_RAMPGEN_H_

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
    volatile uint32_t TRIGGER_RST_RAMPGEN;
    volatile uint32_t RAMGEN_CTRL;
    volatile uint32_t CHIRP_CFG_ADDRESS;
    volatile uint32_t CHIRP_CFG_LOOP_CFG;
    volatile uint32_t RAMPGEN_CALIB_CHIRP_CFG;
    volatile uint32_t RAMPGEN_WR_SPARE2;
    volatile uint32_t LOOP_TIME;
    volatile uint32_t MAIN_PATH_TIME;
    volatile uint32_t FRAME_START_DELAY;
    volatile uint32_t ADC_SAMPLING_DELAY;
    volatile uint32_t BPM_K_COUNT;
    volatile uint32_t BPM_CFG;
    volatile uint32_t BPM_K_CTR_OFFSET;
    volatile uint32_t BPM_PRG_SEQUENCE_1A_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_1A_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_1B_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_1B_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_2A_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_2A_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_2B_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_2B_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_3A_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_3A_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_3B_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_3B_LSW;
    volatile uint32_t BPM_LFSR_POLY_1A;
    volatile uint32_t BPM_LFSR_POLY_1B;
    volatile uint32_t BPM_LFSR_POLY_2A;
    volatile uint32_t BPM_LFSR_POLY_2B;
    volatile uint32_t BPM_LFSR_POLY_3A;
    volatile uint32_t BPM_LFSR_POLY_3B;
    volatile uint32_t BPM_LFSR_INIT_VAL_1A;
    volatile uint32_t BPM_LFSR_INIT_VAL_1B;
    volatile uint32_t BPM_LFSR_INIT_VAL_2A;
    volatile uint32_t BPM_LFSR_INIT_VAL_2B;
    volatile uint32_t BPM_LFSR_INIT_VAL_3A;
    volatile uint32_t BPM_LFSR_INIT_VAL_3B;
    volatile uint32_t LO_START_DELAY;
    volatile uint32_t RX1_2_RF_START_DELTA;
    volatile uint32_t RX3_4_RF_START_DELTA;
    volatile uint32_t RX1_2_BB_START_DELTA;
    volatile uint32_t RX3_4_BB_START_DELTA;
    volatile uint32_t LO_END_DELTA;
    volatile uint32_t RX1_2_RF_END_DELTA;
    volatile uint32_t RX3_4_RF_END_DELTA;
    volatile uint32_t RX1_2_BB_END_DELTA;
    volatile uint32_t RX3_4_BB_END_DELTA;
    volatile uint32_t FREQ_MONITOR_RST_END;
    volatile uint32_t RESET_SOCC_START_END;
    volatile uint32_t RAMPGEN_WR_SPARE3;
    volatile uint32_t N_START_BYP;
    volatile uint32_t N_SLOPE_BYP;
    volatile uint32_t BYPASS_RAMPGEN_CTRL;
    volatile uint32_t RAMPGEN_WR_SPARE4;
    volatile uint32_t SEQ_EXT_DELAY;
    volatile uint32_t TX_REG;
    volatile uint32_t RAM_ECC1;
    volatile uint32_t RAM_ECC2;
    volatile uint32_t SEQ_OVR_CTRL;
    volatile uint32_t SEQ_OVR_VAL;
    volatile uint32_t ADC_ENABLE_RST;
    volatile uint32_t CURRENT_CHIRP_STATUS;
    volatile uint32_t CHIRP_COUNTER;
    volatile uint8_t  Resv_256[4];
    volatile uint32_t RAMPGEN_RD_SPARE2;
    volatile uint32_t RAMPGEN_RD_SPARE3;
    volatile uint32_t RAMPGEN_RD_SPARE4;
    volatile uint32_t RAMPGEN_RD_SPARE5;
    volatile uint32_t SEQ_EXT_REG1_WR;
    volatile uint32_t SEQ_EXT_REG2_WR;
    volatile uint32_t SEQ_EXT_REG3_WR;
    volatile uint32_t SEQ_EXT_REG4_WR;
    volatile uint32_t SEQ_EXT_REG5_WR;
    volatile uint32_t SEQ_EXT_REG6_WR;
    volatile uint32_t SEQ_EXT_REG7_WR;
    volatile uint32_t SEQ_EXT_REG1_RD;
    volatile uint32_t SEQ_EXT_REG2_RD;
    volatile uint32_t SEQ_EXT_REG3_RD;
    volatile uint32_t SEQ_EXT_REG5_RD;
    volatile uint32_t SEQ_EXT_REG8_WR;
    volatile uint32_t SEQ_EXT_REG9_WR;
    volatile uint32_t SYNTH_CYCLE_CNTR_CFG1;
    volatile uint32_t SYNTH_CYCLE_CNTR_OVERRIDE_CFG1;
    volatile uint32_t SYNTH_CYCLE_CNTR_OVERRIDE_CFG2;
    volatile uint32_t SYNTH_CYCLE_CNTR_OVERRIDE_CFG3;
    volatile uint32_t LINEAR_RAM_BASE_ADDR1;
    volatile uint32_t LINEAR_RAM_BASE_ADDR2;
    volatile uint32_t SYNTH_CYCLE_CNTR_CURRENT_RD1;
    volatile uint32_t SYNTH_CYCLE_CNTR_CURRENT_RD2;
    volatile uint32_t SYNTH_CYCLE_CNTR_CURRENT_RD3;
    volatile uint32_t SYNTH_CYCLE_CNTR_CURRENT_RD4;
    volatile uint32_t SYNTH_CYCLE_CNTR_CONTINUOUS_RD1;
    volatile uint32_t SYNTH_CYCLE_CNTR_CONTINUOUS_RD2;
    volatile uint32_t SYNTH_CYCLE_CNTR_CONTINUOUS_RD3;
    volatile uint32_t SYNTH_CYCLE_CNTR_CONTINUOUS_RD4;
    volatile uint32_t SYNTH_CYCLE_CNTR_MISC_RD1;
    volatile uint32_t SYNTH_CYCLE_CNTR_MISC_RD2;
    volatile uint32_t SOCC_OVERRIDE;
    volatile uint32_t SEQ_EXT_REG4_RD;
    volatile uint32_t LINEAR_RAM_INIT;
    volatile uint32_t SOCC_8FF_VALID_FREQ_RANGE1_MIN_MAX;
    volatile uint32_t SOCC_8FF_VALID_FREQ_RANGE2_MIN_MAX;
    volatile uint32_t SOCC_7FF_VALID_FREQ_RANGE1_MIN_MAX;
    volatile uint32_t SOCC_7FF_VALID_FREQ_RANGE2_MIN_MAX;
    volatile uint32_t SOCC_5FF_VALID_FREQ_RANGE1_MIN_MAX;
    volatile uint32_t SOCC_5FF_VALID_FREQ_RANGE2_MIN_MAX;
    volatile uint8_t  Resv_448[24];
    volatile uint32_t SOCC_RING_LEN_CFG_SEL;
    volatile uint8_t  Resv_460[8];
    volatile uint32_t CRD_MAX_FREQ_OFFSET_DITHER;
    volatile uint32_t CRD_HIGH_BW_START_TIME;
    volatile uint32_t CRD_HIGH_BW_STOP_TIME;
    volatile uint32_t CRD_LFSR_SEED;
    volatile uint32_t CRD_LFSR_SEED_CTRL;
    volatile uint32_t BPM_LFSR_INIT_VAL_4A;
    volatile uint32_t BPM_LFSR_INIT_VAL_4B;
    volatile uint32_t BPM_PRG_SEQUENCE_4A_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_4A_LSW;
    volatile uint32_t BPM_PRG_SEQUENCE_4B_MSW;
    volatile uint32_t BPM_PRG_SEQUENCE_4B_LSW;
    volatile uint32_t BPM_LFSR_POLY_4A;
    volatile uint32_t BPM_LFSR_POLY_4B;
} CSL_bss_rampgenRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN                                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL                                            (0x00000004U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS                                      (0x00000008U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG                                     (0x0000000CU)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG                                (0x00000010U)
#define CSL_BSS_RAMPGEN_RAMPGEN_WR_SPARE2                                      (0x00000014U)
#define CSL_BSS_RAMPGEN_LOOP_TIME                                              (0x00000018U)
#define CSL_BSS_RAMPGEN_MAIN_PATH_TIME                                         (0x0000001CU)
#define CSL_BSS_RAMPGEN_FRAME_START_DELAY                                      (0x00000020U)
#define CSL_BSS_RAMPGEN_ADC_SAMPLING_DELAY                                     (0x00000024U)
#define CSL_BSS_RAMPGEN_BPM_K_COUNT                                            (0x00000028U)
#define CSL_BSS_RAMPGEN_BPM_CFG                                                (0x0000002CU)
#define CSL_BSS_RAMPGEN_BPM_K_CTR_OFFSET                                       (0x00000030U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_1A_MSW                                (0x00000034U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_1A_LSW                                (0x00000038U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_1B_MSW                                (0x0000003CU)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_1B_LSW                                (0x00000040U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_2A_MSW                                (0x00000044U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_2A_LSW                                (0x00000048U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_2B_MSW                                (0x0000004CU)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_2B_LSW                                (0x00000050U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_3A_MSW                                (0x00000054U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_3A_LSW                                (0x00000058U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_3B_MSW                                (0x0000005CU)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_3B_LSW                                (0x00000060U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_1A                                       (0x00000064U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_1B                                       (0x00000068U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_2A                                       (0x0000006CU)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_2B                                       (0x00000070U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_3A                                       (0x00000074U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_3B                                       (0x00000078U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_1A                                   (0x0000007CU)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_1B                                   (0x00000080U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_2A                                   (0x00000084U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_2B                                   (0x00000088U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_3A                                   (0x0000008CU)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_3B                                   (0x00000090U)
#define CSL_BSS_RAMPGEN_LO_START_DELAY                                         (0x00000094U)
#define CSL_BSS_RAMPGEN_RX1_2_RF_START_DELTA                                   (0x00000098U)
#define CSL_BSS_RAMPGEN_RX3_4_RF_START_DELTA                                   (0x0000009CU)
#define CSL_BSS_RAMPGEN_RX1_2_BB_START_DELTA                                   (0x000000A0U)
#define CSL_BSS_RAMPGEN_RX3_4_BB_START_DELTA                                   (0x000000A4U)
#define CSL_BSS_RAMPGEN_LO_END_DELTA                                           (0x000000A8U)
#define CSL_BSS_RAMPGEN_RX1_2_RF_END_DELTA                                     (0x000000ACU)
#define CSL_BSS_RAMPGEN_RX3_4_RF_END_DELTA                                     (0x000000B0U)
#define CSL_BSS_RAMPGEN_RX1_2_BB_END_DELTA                                     (0x000000B4U)
#define CSL_BSS_RAMPGEN_RX3_4_BB_END_DELTA                                     (0x000000B8U)
#define CSL_BSS_RAMPGEN_FREQ_MONITOR_RST_END                                   (0x000000BCU)
#define CSL_BSS_RAMPGEN_RESET_SOCC_START_END                                   (0x000000C0U)
#define CSL_BSS_RAMPGEN_RAMPGEN_WR_SPARE3                                      (0x000000C4U)
#define CSL_BSS_RAMPGEN_N_START_BYP                                            (0x000000C8U)
#define CSL_BSS_RAMPGEN_N_SLOPE_BYP                                            (0x000000CCU)
#define CSL_BSS_RAMPGEN_BYPASS_RAMPGEN_CTRL                                    (0x000000D0U)
#define CSL_BSS_RAMPGEN_RAMPGEN_WR_SPARE4                                      (0x000000D4U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_DELAY                                          (0x000000D8U)
#define CSL_BSS_RAMPGEN_TX_REG                                                 (0x000000DCU)
#define CSL_BSS_RAMPGEN_RAM_ECC1                                               (0x000000E0U)
#define CSL_BSS_RAMPGEN_RAM_ECC2                                               (0x000000E4U)
#define CSL_BSS_RAMPGEN_SEQ_OVR_CTRL                                           (0x000000E8U)
#define CSL_BSS_RAMPGEN_SEQ_OVR_VAL                                            (0x000000ECU)
#define CSL_BSS_RAMPGEN_ADC_ENABLE_RST                                         (0x000000F0U)
#define CSL_BSS_RAMPGEN_CURRENT_CHIRP_STATUS                                   (0x000000F4U)
#define CSL_BSS_RAMPGEN_CHIRP_COUNTER                                          (0x000000F8U)
#define CSL_BSS_RAMPGEN_RAMPGEN_RD_SPARE2                                      (0x00000100U)
#define CSL_BSS_RAMPGEN_RAMPGEN_RD_SPARE3                                      (0x00000104U)
#define CSL_BSS_RAMPGEN_RAMPGEN_RD_SPARE4                                      (0x00000108U)
#define CSL_BSS_RAMPGEN_RAMPGEN_RD_SPARE5                                      (0x0000010CU)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG1_WR                                        (0x00000110U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG2_WR                                        (0x00000114U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG3_WR                                        (0x00000118U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG4_WR                                        (0x0000011CU)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG5_WR                                        (0x00000120U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG6_WR                                        (0x00000124U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG7_WR                                        (0x00000128U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG1_RD                                        (0x0000012CU)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG2_RD                                        (0x00000130U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG3_RD                                        (0x00000134U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG5_RD                                        (0x00000138U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG8_WR                                        (0x0000013CU)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG9_WR                                        (0x00000140U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CFG1                                  (0x00000144U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_OVERRIDE_CFG1                         (0x00000148U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_OVERRIDE_CFG2                         (0x0000014CU)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_OVERRIDE_CFG3                         (0x00000150U)
#define CSL_BSS_RAMPGEN_LINEAR_RAM_BASE_ADDR1                                  (0x00000154U)
#define CSL_BSS_RAMPGEN_LINEAR_RAM_BASE_ADDR2                                  (0x00000158U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CURRENT_RD1                           (0x0000015CU)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CURRENT_RD2                           (0x00000160U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CURRENT_RD3                           (0x00000164U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CURRENT_RD4                           (0x00000168U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CONTINUOUS_RD1                        (0x0000016CU)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CONTINUOUS_RD2                        (0x00000170U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CONTINUOUS_RD3                        (0x00000174U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_CONTINUOUS_RD4                        (0x00000178U)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_MISC_RD1                              (0x0000017CU)
#define CSL_BSS_RAMPGEN_SYNTH_CYCLE_CNTR_MISC_RD2                              (0x00000180U)
#define CSL_BSS_RAMPGEN_SOCC_OVERRIDE                                          (0x00000184U)
#define CSL_BSS_RAMPGEN_SEQ_EXT_REG4_RD                                        (0x00000188U)
#define CSL_BSS_RAMPGEN_LINEAR_RAM_INIT                                        (0x0000018CU)
#define CSL_BSS_RAMPGEN_SOCC_8FF_VALID_FREQ_RANGE1_MIN_MAX                     (0x00000190U)
#define CSL_BSS_RAMPGEN_SOCC_8FF_VALID_FREQ_RANGE2_MIN_MAX                     (0x00000194U)
#define CSL_BSS_RAMPGEN_SOCC_7FF_VALID_FREQ_RANGE1_MIN_MAX                     (0x00000198U)
#define CSL_BSS_RAMPGEN_SOCC_7FF_VALID_FREQ_RANGE2_MIN_MAX                     (0x0000019CU)
#define CSL_BSS_RAMPGEN_SOCC_5FF_VALID_FREQ_RANGE1_MIN_MAX                     (0x000001A0U)
#define CSL_BSS_RAMPGEN_SOCC_5FF_VALID_FREQ_RANGE2_MIN_MAX                     (0x000001A4U)
#define CSL_BSS_RAMPGEN_SOCC_RING_LEN_CFG_SEL                                  (0x000001C0U)
#define CSL_BSS_RAMPGEN_CRD_MAX_FREQ_OFFSET_DITHER                             (0x000001CCU)
#define CSL_BSS_RAMPGEN_CRD_HIGH_BW_START_TIME                                 (0x000001D0U)
#define CSL_BSS_RAMPGEN_CRD_HIGH_BW_STOP_TIME                                  (0x000001D4U)
#define CSL_BSS_RAMPGEN_CRD_LFSR_SEED                                          (0x000001D8U)
#define CSL_BSS_RAMPGEN_CRD_LFSR_SEED_CTRL                                     (0x000001DCU)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_4A                                   (0x000001E0U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_INIT_VAL_4B                                   (0x000001E4U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_4A_MSW                                (0x000001E8U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_4A_LSW                                (0x000001ECU)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_4B_MSW                                (0x000001F0U)
#define CSL_BSS_RAMPGEN_BPM_PRG_SEQUENCE_4B_LSW                                (0x000001F4U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_4A                                       (0x000001F8U)
#define CSL_BSS_RAMPGEN_BPM_LFSR_POLY_4B                                       (0x000001FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TRIGGER_RST_RAMPGEN */

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RAMP_GEN_TRIGGER_MASK              (0x00000001U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RAMP_GEN_TRIGGER_SHIFT             (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RAMP_GEN_TRIGGER_RESETVAL          (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RAMP_GEN_TRIGGER_MAX               (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU1_MASK                           (0x000000FEU)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU1_SHIFT                          (0x00000001U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU1_MAX                            (0x0000007FU)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RESET_RAMP_GEN_MASK                (0x00000100U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RESET_RAMP_GEN_SHIFT               (0x00000008U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RESET_RAMP_GEN_RESETVAL            (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RESET_RAMP_GEN_MAX                 (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU2_MASK                           (0x0000FE00U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU2_SHIFT                          (0x00000009U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU2_MAX                            (0x0000007FU)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_LOCKSTEP_MASK               (0x00010000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_LOCKSTEP_SHIFT              (0x00000010U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_LOCKSTEP_RESETVAL           (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_LOCKSTEP_MAX                (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_MASK          (0x00020000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_SHIFT         (0x00000011U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_RESETVAL      (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_MAX           (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU3_MASK                           (0x00FC0000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU3_SHIFT                          (0x00000012U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU3_RESETVAL                       (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU3_MAX                            (0x0000003FU)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_SELFTEST_LOCKSTEP_MASK      (0x07000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_SELFTEST_LOCKSTEP_SHIFT     (0x00000018U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_SELFTEST_LOCKSTEP_RESETVAL  (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_SELFTEST_LOCKSTEP_MAX       (0x00000007U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU4_MASK                           (0x08000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU4_SHIFT                          (0x0000001BU)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU4_RESETVAL                       (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU4_MAX                            (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_INV_MASK      (0x70000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_INV_SHIFT     (0x0000001CU)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_INV_RESETVAL  (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_ENABLE_PARITY_RAMOUT_INV_MAX       (0x00000007U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU5_MASK                           (0x80000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU5_SHIFT                          (0x0000001FU)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU5_RESETVAL                       (0x00000000U)
#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_NU5_MAX                            (0x00000001U)

#define CSL_BSS_RAMPGEN_TRIGGER_RST_RAMPGEN_RESETVAL                           (0x00000000U)

/* RAMGEN_CTRL */

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_OPEN_LOOP_MASK                     (0x00000001U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_OPEN_LOOP_SHIFT                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_OPEN_LOOP_RESETVAL                 (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_OPEN_LOOP_MAX                      (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_FREEZE_MAIN_PATH_MASK              (0x00000002U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_FREEZE_MAIN_PATH_SHIFT             (0x00000001U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_FREEZE_MAIN_PATH_RESETVAL          (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_DISABLE_FREEZE_MAIN_PATH_MAX               (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_ON_MASK                            (0x00000004U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_ON_SHIFT                           (0x00000002U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_ON_RESETVAL                        (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_ON_MAX                             (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX_LO_ON_MASK                         (0x00000008U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX_LO_ON_SHIFT                        (0x00000003U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX_LO_ON_RESETVAL                     (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX_LO_ON_MAX                          (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_LO_ON_MASK                         (0x00000010U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_LO_ON_SHIFT                        (0x00000004U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_LO_ON_RESETVAL                     (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_TX_LO_ON_MAX                          (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_RF_ON_MASK                        (0x00000020U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_RF_ON_SHIFT                       (0x00000005U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_RF_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_RF_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_RF_ON_MASK                        (0x00000040U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_RF_ON_SHIFT                       (0x00000006U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_RF_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_RF_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_RF_ON_MASK                        (0x00000080U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_RF_ON_SHIFT                       (0x00000007U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_RF_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_RF_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_RF_ON_MASK                        (0x00000100U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_RF_ON_SHIFT                       (0x00000008U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_RF_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_RF_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_BB_ON_MASK                        (0x00000200U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_BB_ON_SHIFT                       (0x00000009U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_BB_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX1_BB_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_BB_ON_MASK                        (0x00000400U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_BB_ON_SHIFT                       (0x0000000AU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_BB_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX2_BB_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_BB_ON_MASK                        (0x00000800U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_BB_ON_SHIFT                       (0x0000000BU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_BB_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX3_BB_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_BB_ON_MASK                        (0x00001000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_BB_ON_SHIFT                       (0x0000000CU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_BB_ON_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_KEEP_RX4_BB_ON_MAX                         (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_FORCE_FREEZE_MAIN_PATH_MASK                (0x00002000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_FORCE_FREEZE_MAIN_PATH_SHIFT               (0x0000000DU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_FORCE_FREEZE_MAIN_PATH_RESETVAL            (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_FORCE_FREEZE_MAIN_PATH_MAX                 (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_TRIGGER_MODE_MASK                 (0x00004000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_TRIGGER_MODE_SHIFT                (0x0000000EU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_TRIGGER_MODE_RESETVAL             (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_TRIGGER_MODE_MAX                  (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_MASTER_ENABLE_MASK                (0x00008000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_MASTER_ENABLE_SHIFT               (0x0000000FU)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_MASTER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RAMP_GEN_MASTER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_NU2_MASK                                   (0xFFFF0000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_NU2_SHIFT                                  (0x00000010U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_NU2_MAX                                    (0x0000FFFFU)

#define CSL_BSS_RAMPGEN_RAMGEN_CTRL_RESETVAL                                   (0x00000000U)

/* CHIRP_CFG_ADDRESS */

#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_FIRST_CHIRP_ADDRESS_MASK             (0x000001FFU)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_FIRST_CHIRP_ADDRESS_SHIFT            (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_FIRST_CHIRP_ADDRESS_RESETVAL         (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_FIRST_CHIRP_ADDRESS_MAX              (0x000001FFU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU1_MASK                             (0x0000FE00U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU1_SHIFT                            (0x00000009U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU1_MAX                              (0x0000007FU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_LAST_CHIRP_ADDRESS_MASK              (0x01FF0000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_LAST_CHIRP_ADDRESS_SHIFT             (0x00000010U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_LAST_CHIRP_ADDRESS_RESETVAL          (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_LAST_CHIRP_ADDRESS_MAX               (0x000001FFU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_ADDRESS_RESETVAL                             (0x00000000U)

/* CHIRP_CFG_LOOP_CFG */

#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_N_LOOPS_MASK                        (0x000000FFU)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_N_LOOPS_SHIFT                       (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_N_LOOPS_RESETVAL                    (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_N_LOOPS_MAX                         (0x000000FFU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU1_MASK                            (0x0000FF00U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU1_SHIFT                           (0x00000008U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU1_RESETVAL                        (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU1_MAX                             (0x000000FFU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_ENDLESS_FRAME_MODE_MASK             (0x00010000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_ENDLESS_FRAME_MODE_SHIFT            (0x00000010U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_ENDLESS_FRAME_MODE_RESETVAL         (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_ENDLESS_FRAME_MODE_MAX              (0x00000001U)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU2_MASK                            (0xFFFE0000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU2_SHIFT                           (0x00000011U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU2_RESETVAL                        (0x00000000U)
#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_NU2_MAX                             (0x00007FFFU)

#define CSL_BSS_RAMPGEN_CHIRP_CFG_LOOP_CFG_RESETVAL                            (0x00000000U)

/* RAMPGEN_CALIB_CHIRP_CFG */

#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_ENABLE_MASK              (0x00000001U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_ENABLE_SHIFT             (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_ENABLE_RESETVAL          (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_ENABLE_MAX               (0x00000001U)

#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU1_MASK                       (0x000000FEU)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU1_SHIFT                      (0x00000001U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU1_RESETVAL                   (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU1_MAX                        (0x0000007FU)

#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_CHIRP_PROFILE_SELECT_MASK (0x00000F00U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_CHIRP_PROFILE_SELECT_SHIFT (0x00000008U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_CHIRP_PROFILE_SELECT_RESETVAL (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_CALIB_CHIRP_PROFILE_SELECT_MAX (0x0000000FU)

#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU2_MASK                       (0x0000F000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU2_SHIFT                      (0x0000000CU)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU2_RESETVAL                   (0x00000000U)
#define CSL_BSS_RAMPGEN_RAMPGEN_CALIB_CHIRP_CFG_NU2_MAX                        (0x0000000FU)

#ifdef __cplusplus
}
#endif
#endif /* #ifndef CSLR_BSS_RAMPGEN_H_ */

