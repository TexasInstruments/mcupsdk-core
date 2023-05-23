/********************************************************************
* Copyright (C) 2024 Texas Instruments Incorporated.
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
*/
#ifndef CSLR_INTR_ESM1_H_
#define CSLR_INTR_ESM1_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* List of intr sources for receiver: esm1 */
/* This instance name corresponds to design instance name: esm_mcu_mcu_0 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_0_LOSSREF_IPCFG    (0U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_0_lossref_ipcfg_0, Dest signal name: esm_lvl_event_0 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_0_PHASELOCK_N (1U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_0_phaselock_n_0, Dest signal name: esm_lvl_event_1 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_0_FREQLOCK_N  (2U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_0_freqlock_n_0, Dest signal name: esm_lvl_event_2 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_0_HIGHJITTER_IPCFG (3U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_0_highjitter_ipcfg_0, Dest signal name: esm_lvl_event_3 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_0_RECAL_IPCFG (4U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_0_recal_ipcfg_0, Dest signal name: esm_lvl_event_4 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_1_LOSSREF_IPCFG    (5U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_1_lossref_ipcfg_0, Dest signal name: esm_lvl_event_5 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_1_PHASELOCK_N (6U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_1_phaselock_n_0, Dest signal name: esm_lvl_event_6 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_1_FREQLOCK_N  (7U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_1_freqlock_n_0, Dest signal name: esm_lvl_event_7 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_1_HIGHJITTER_IPCFG (8U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_1_highjitter_ipcfg_0, Dest signal name: esm_lvl_event_8 */
#define CSL_ESM1_INTR_ADPLLM_HSDIV_WRAP_ADPLLM_HSDIV_WRAP_MCU_1_RECAL_IPCFG (9U)  /* Design details: Source signal name: adpllm_hsdiv_wrap_mcu_1_recal_ipcfg_0, Dest signal name: esm_lvl_event_9 */
#define CSL_ESM1_INTR_ADC0_ECC_CORRECTED_ERR_LEVEL                    (10U)  /* Design details: Source signal name: adc12_gs80_mcu_0_ecc_corrected_err_level_0, Dest signal name: esm_lvl_event_10 */
#define CSL_ESM1_INTR_ADC0_ECC_UNCORRECTED_ERR_LEVEL                  (11U)  /* Design details: Source signal name: adc12_gs80_mcu_0_ecc_uncorrected_err_level_0, Dest signal name: esm_lvl_event_11 */
#define CSL_ESM1_INTR_ADC1_ECC_CORRECTED_ERR_LEVEL                    (12U)  /* Design details: Source signal name: adc12_gs80_mcu_1_ecc_corrected_err_level_0, Dest signal name: esm_lvl_event_12 */
#define CSL_ESM1_INTR_ADC1_ECC_UNCORRECTED_ERR_LEVEL                  (13U)  /* Design details: Source signal name: adc12_gs80_mcu_1_ecc_uncorrected_err_level_0, Dest signal name: esm_lvl_event_13 */
#define CSL_ESM1_INTR_CPSW0_ECC_SEC_PEND                              (14U)  /* Design details: Source signal name: cpsw_2guss_mcu_0_ecc_sec_pend_0, Dest signal name: esm_lvl_event_14 */
#define CSL_ESM1_INTR_CPSW0_ECC_DED_PEND                              (15U)  /* Design details: Source signal name: cpsw_2guss_mcu_0_ecc_ded_pend_0, Dest signal name: esm_lvl_event_15 */
#define CSL_ESM1_INTR_MCAN0_MCANSS_ECC_CORR_LVL_INT                   (16U)  /* Design details: Source signal name: mcanss_mcu_0_mcanss_ecc_corr_lvl_int_0, Dest signal name: esm_lvl_event_16 */
#define CSL_ESM1_INTR_MCAN0_MCANSS_ECC_UNCORR_LVL_INT                 (17U)  /* Design details: Source signal name: mcanss_mcu_0_mcanss_ecc_uncorr_lvl_int_0, Dest signal name: esm_lvl_event_17 */
#define CSL_ESM1_INTR_MCAN1_MCANSS_ECC_CORR_LVL_INT                   (18U)  /* Design details: Source signal name: mcanss_mcu_1_mcanss_ecc_corr_lvl_int_0, Dest signal name: esm_lvl_event_18 */
#define CSL_ESM1_INTR_MCAN1_MCANSS_ECC_UNCORR_LVL_INT                 (19U)  /* Design details: Source signal name: mcanss_mcu_1_mcanss_ecc_uncorr_lvl_int_0, Dest signal name: esm_lvl_event_19 */
#define CSL_ESM1_INTR_FSS0_OSPI0_ECC_CORR_LVL_INTR                    (24U)  /* Design details: Source signal name: fss_mcu_0_ospi0_ecc_corr_lvl_intr_0, Dest signal name: esm_lvl_event_24 */
#define CSL_ESM1_INTR_FSS0_OSPI0_ECC_UNCORR_LVL_INTR                  (25U)  /* Design details: Source signal name: fss_mcu_0_ospi0_ecc_uncorr_lvl_intr_0, Dest signal name: esm_lvl_event_25 */
#define CSL_ESM1_INTR_FSS0_OSPI1_ECC_CORR_LVL_INTR                    (26U)  /* Design details: Source signal name: fss_mcu_0_ospi1_ecc_corr_lvl_intr_0, Dest signal name: esm_lvl_event_26 */
#define CSL_ESM1_INTR_FSS0_OSPI1_ECC_UNCORR_LVL_INTR                  (27U)  /* Design details: Source signal name: fss_mcu_0_ospi1_ecc_uncorr_lvl_intr_0, Dest signal name: esm_lvl_event_27 */
#define CSL_ESM1_INTR_FSS0_HPB_ECC_CORR_LEVEL                         (28U)  /* Design details: Source signal name: fss_mcu_0_hpb_ecc_corr_level_0, Dest signal name: esm_lvl_event_28 */
#define CSL_ESM1_INTR_FSS0_HPB_ECC_UNCORR_LEVEL                       (29U)  /* Design details: Source signal name: fss_mcu_0_hpb_ecc_uncorr_level_0, Dest signal name: esm_lvl_event_29 */
#define CSL_ESM1_INTR_FSS0_ECC_INTR_ERR_PEND_0                        (30U)  /* Design details: Source signal name: fss_mcu_0_ecc_intr_err_pend_0, Dest signal name: esm_lvl_event_30 */
#define CSL_ESM1_INTR_FSS0_FSAS_ECC_INTR_ERR_PEND                     (31U)  /* Design details: Source signal name: fss_mcu_0_fsas_ecc_intr_err_pend_0, Dest signal name: esm_lvl_event_31 */
#define CSL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL                   (32U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_ecc_corrected_level_0, Dest signal name: esm_lvl_event_32 */
#define CSL_ESM1_INTR_MCU0_CPU0_ECC_UNCORRECTED_LEVEL                 (33U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_ecc_uncorrected_level_0, Dest signal name: esm_lvl_event_33 */
#define CSL_ESM1_INTR_MCU0_CPU1_ECC_CORRECTED_LEVEL                   (34U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu1_ecc_corrected_level_0, Dest signal name: esm_lvl_event_34 */
#define CSL_ESM1_INTR_MCU0_CPU1_ECC_UNCORRECTED_LEVEL                 (35U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu1_ecc_uncorrected_level_0, Dest signal name: esm_lvl_event_35 */
#define CSL_ESM1_INTR_MCU0_CPU0_EXP_INTR_0                            (36U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_exp_intr_0, Dest signal name: esm_lvl_event_36 */
#define CSL_ESM1_INTR_MCU0_CPU1_EXP_INTR_0                            (37U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu1_exp_intr_0, Dest signal name: esm_lvl_event_37 */
#define CSL_ESM1_INTR_NAVSS0_MODSS_ECC_SEC_PEND                       (40U)  /* Design details: Source signal name: navss_mcu_mcu_0_modss_ecc_sec_pend_0, Dest signal name: esm_lvl_event_40 */
#define CSL_ESM1_INTR_NAVSS0_MODSS_ECC_DED_PEND                       (41U)  /* Design details: Source signal name: navss_mcu_mcu_0_modss_ecc_ded_pend_0, Dest signal name: esm_lvl_event_41 */
#define CSL_ESM1_INTR_NAVSS0_UDMASS_ECC_SEC_PEND                      (42U)  /* Design details: Source signal name: navss_mcu_mcu_0_udmass_ecc_sec_pend_0, Dest signal name: esm_lvl_event_42 */
#define CSL_ESM1_INTR_NAVSS0_UDMASS_ECC_DED_PEND                      (43U)  /* Design details: Source signal name: navss_mcu_mcu_0_udmass_ecc_ded_pend_0, Dest signal name: esm_lvl_event_43 */
#define CSL_ESM1_INTR_PDMA_MCU0_PDMA_MCU0_MCU_0_ECC_SEC_PEND          (48U)  /* Design details: Source signal name: pdma_mcu0_mcu_0_ecc_sec_pend_0, Dest signal name: esm_lvl_event_48 */
#define CSL_ESM1_INTR_PDMA_MCU0_PDMA_MCU0_MCU_0_ECC_DED_PEND          (49U)  /* Design details: Source signal name: pdma_mcu0_mcu_0_ecc_ded_pend_0, Dest signal name: esm_lvl_event_49 */
#define CSL_ESM1_INTR_PDMA_MCU1_PDMA_MCU1_MCU_0_ECC_SEC_PEND          (50U)  /* Design details: Source signal name: pdma_mcu1_mcu_0_ecc_sec_pend_0, Dest signal name: esm_lvl_event_50 */
#define CSL_ESM1_INTR_PDMA_MCU1_PDMA_MCU1_MCU_0_ECC_DED_PEND          (51U)  /* Design details: Source signal name: pdma_mcu1_mcu_0_ecc_ded_pend_0, Dest signal name: esm_lvl_event_51 */
#define CSL_ESM1_INTR_TIMER0_INTR_PEND                                (54U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_0_intr_pend_0, Dest signal name: esm_lvl_event_54 */
#define CSL_ESM1_INTR_TIMER1_INTR_PEND                                (55U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_1_intr_pend_0, Dest signal name: esm_lvl_event_55 */
#define CSL_ESM1_INTR_TIMER2_INTR_PEND                                (56U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_2_intr_pend_0, Dest signal name: esm_lvl_event_56 */
#define CSL_ESM1_INTR_TIMER3_INTR_PEND                                (57U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_3_intr_pend_0, Dest signal name: esm_lvl_event_57 */
#define CSL_ESM1_INTR_DCC0_INTR_ERR_LEVEL                             (60U)  /* Design details: Source signal name: dcc_mcu_0_intr_err_level_0, Dest signal name: esm_lvl_event_60 */
#define CSL_ESM1_INTR_DCC1_INTR_ERR_LEVEL                             (61U)  /* Design details: Source signal name: dcc_mcu_1_intr_err_level_0, Dest signal name: esm_lvl_event_61 */
#define CSL_ESM1_INTR_DCC2_INTR_ERR_LEVEL                             (62U)  /* Design details: Source signal name: dcc_mcu_2_intr_err_level_0, Dest signal name: esm_lvl_event_62 */
#define CSL_ESM1_INTR_SRAM_MCU0_ECC_CORR_LEVEL                        (64U)  /* Design details: Source signal name: msram64kx64e_mcu_0_ecc_corr_level_0, Dest signal name: esm_lvl_event_64 */
#define CSL_ESM1_INTR_SRAM_MCU0_ECC_UNCORR_LEVEL                      (65U)  /* Design details: Source signal name: msram64kx64e_mcu_0_ecc_uncorr_level_0, Dest signal name: esm_lvl_event_65 */
#define CSL_ESM1_INTR_M4_MCUCLK2_ECC_AGGR_M4_MCUCLK2_ECC_AGGR_MCU_0_CORR_LEVEL_0     (68U)  /* Design details: Source signal name: m4_mcuclk2_ecc_aggr_mcu_0_corr_level_0, Dest signal name: esm_lvl_event_68 */
#define CSL_ESM1_INTR_M4_MCUCLK2_ECC_AGGR_M4_MCUCLK2_ECC_AGGR_MCU_0_UNCORR_LEVEL     (69U)  /* Design details: Source signal name: m4_mcuclk2_ecc_aggr_mcu_0_uncorr_level_0, Dest signal name: esm_lvl_event_69 */
#define CSL_ESM1_INTR_M4_MCUCLK4_ECC_AGGR_M4_MCUCLK4_ECC_AGGR_MCU_0_CORR_LEVEL_0     (70U)  /* Design details: Source signal name: m4_mcuclk4_ecc_aggr_mcu_0_corr_level_0, Dest signal name: esm_lvl_event_70 */
#define CSL_ESM1_INTR_M4_MCUCLK4_ECC_AGGR_M4_MCUCLK4_ECC_AGGR_MCU_0_UNCORR_LEVEL     (71U)  /* Design details: Source signal name: m4_mcuclk4_ecc_aggr_mcu_0_uncorr_level_0, Dest signal name: esm_lvl_event_71 */
#define CSL_ESM1_INTR_PBIST0_DFT_PBIST_SAFETY_ERROR                   (74U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_mcu_0_dft_pbist_safety_error_0, Dest signal name: esm_lvl_event_74 */
#define CSL_ESM1_INTR_GLUE_ESM_MAIN_ERR_I_N_GLUE_ESM_MAIN_ERR_I_N_ESM_MAIN_ERR_I_N   (95U)  /* Design details: Source signal name: GLUE_esm_main_err_i_n_esm_main_err_i_n_0, Dest signal name: esm_lvl_event_95 */
#define CSL_ESM1_MCU0_SELFTEST_ERR_INTR                               (96U)   /* Dest signal name: esm_pls_event_0 */
#define CSL_ESM1_MCU0_CPU_CMP_ERR_INTR                                (97U)   /* Dest signal name: esm_pls_event_1 */
#define CSL_ESM1_MCU0_BUS_MON_ERR_INTR                                (98U)   /* Dest signal name: esm_pls_event_2 */
#define CSL_ESM1_MCU0_VIM_CMP_ERR_INTR                                (99U)   /* Dest signal name: esm_pls_event_3 */
#define CSL_ESM1_MCU0_CCM_CMP_ERR_INTR                                (100U)  /* Dest signal name: esm_pls_event_4 */
#define CSL_ESM1_INTR_RTI0_INTR_WWD                                   (104U)  /* Design details: Source signal name: rti_mcu_0_intr_wwd_0, Dest signal name: esm_pls_event_8 */
#define CSL_ESM1_INTR_RTI1_INTR_WWD                                   (105U)  /* Design details: Source signal name: rti_mcu_1_intr_wwd_0, Dest signal name: esm_pls_event_9 */

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_ESM1_H_ */
