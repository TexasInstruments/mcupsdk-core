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
#ifndef CSLR_INTR_ESM0_H_
#define CSLR_INTR_ESM0_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* List of intr sources for receiver: esm0 */
/* This instance name corresponds to design instance name: esm_main_main_0 */
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_LOSSREF_IPCFG     (0U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_0_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_0*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_PHASELOCK_N  (1U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_0_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_1*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_FREQLOCK_N   (2U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_0_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_2*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HIGHJITTER_IPCFG  (3U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_0_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_3*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_1_BUS_LOSSREF_IPCFG  (5U)  /* Design details: Source signal name: adpllljm_wrap_main_1_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_5*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_1_BUS_PHASELOCK_N    (6U)  /* Design details: Source signal name: adpllljm_wrap_main_1_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_6*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_1_BUS_FREQLOCK_N     (7U)  /* Design details: Source signal name: adpllljm_wrap_main_1_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_7*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_1_BUS_HIGHJITTER_IPCFG    (8U)  /* Design details: Source signal name: adpllljm_wrap_main_1_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_8*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_2_BUS_LOSSREF_IPCFG    (10U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_2_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_10*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_2_BUS_PHASELOCK_N (11U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_2_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_11*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_2_BUS_FREQLOCK_N  (12U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_2_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_12*/
#define CSL_ESM0_INTR_ADPLLLJM_HSDIV_WRAP_ADPLLLJM_HSDIV_WRAP_MAIN_2_BUS_HIGHJITTER_IPCFG (13U)  /* Design details: Source signal name: adpllljm_hsdiv_wrap_main_2_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_13*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_3_BUS_LOSSREF_IPCFG (15U)  /* Design details: Source signal name: adpllljm_wrap_main_3_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_15*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_3_BUS_PHASELOCK_N   (16U)  /* Design details: Source signal name: adpllljm_wrap_main_3_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_16*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_3_BUS_FREQLOCK_N    (17U)  /* Design details: Source signal name: adpllljm_wrap_main_3_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_17*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_3_BUS_HIGHJITTER_IPCFG   (18U)  /* Design details: Source signal name: adpllljm_wrap_main_3_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_18*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_4_BUS_LOSSREF_IPCFG (20U)  /* Design details: Source signal name: adpllljm_wrap_main_4_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_20*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_4_BUS_PHASELOCK_N   (21U)  /* Design details: Source signal name: adpllljm_wrap_main_4_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_21*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_4_BUS_FREQLOCK_N    (22U)  /* Design details: Source signal name: adpllljm_wrap_main_4_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_22*/
#define CSL_ESM0_INTR_ADPLLLJM_WRAP_ADPLLLJM_WRAP_MAIN_4_BUS_HIGHJITTER_IPCFG   (23U)  /* Design details: Source signal name: adpllljm_wrap_main_4_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_23*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_6_BUS_LOSSREF_IPCFG     (30U)  /* Design details: Source signal name: adpllm_wrap_main_6_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_30*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_6_BUS_PHASELOCK_N  (31U)  /* Design details: Source signal name: adpllm_wrap_main_6_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_31*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_6_BUS_FREQLOCK_N   (32U)  /* Design details: Source signal name: adpllm_wrap_main_6_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_32*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_6_BUS_HIGHJITTER_IPCFG  (33U)  /* Design details: Source signal name: adpllm_wrap_main_6_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_33*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_6_BUS_RECAL_IPCFG  (34U)  /* Design details: Source signal name: adpllm_wrap_main_6_bus_recal_ipcfg_0, Dest signal name: bus_esm_lvl_event_34*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_7_BUS_LOSSREF_IPCFG     (35U)  /* Design details: Source signal name: adpllm_wrap_main_7_bus_lossref_ipcfg_0, Dest signal name: bus_esm_lvl_event_35*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_7_BUS_PHASELOCK_N  (36U)  /* Design details: Source signal name: adpllm_wrap_main_7_bus_phaselock_n_0, Dest signal name: bus_esm_lvl_event_36*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_7_BUS_FREQLOCK_N   (37U)  /* Design details: Source signal name: adpllm_wrap_main_7_bus_freqlock_n_0, Dest signal name: bus_esm_lvl_event_37*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_7_BUS_HIGHJITTER_IPCFG  (38U)  /* Design details: Source signal name: adpllm_wrap_main_7_bus_highjitter_ipcfg_0, Dest signal name: bus_esm_lvl_event_38*/
#define CSL_ESM0_INTR_ADPLLM_WRAP_ADPLLM_WRAP_MAIN_7_BUS_RECAL_IPCFG  (39U)  /* Design details: Source signal name: adpllm_wrap_main_7_bus_recal_ipcfg_0, Dest signal name: bus_esm_lvl_event_39*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ARM0_INTERRIRQ             (40U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_arm0_interrirq_0, Dest signal name: bus_esm_lvl_event_40*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ARM0_EXTERRIRQ             (41U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_arm0_exterrirq_0, Dest signal name: bus_esm_lvl_event_41*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ARM1_INTERRIRQ             (42U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_arm1_interrirq_0, Dest signal name: bus_esm_lvl_event_42*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ARM1_EXTERRIRQ             (43U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_arm1_exterrirq_0, Dest signal name: bus_esm_lvl_event_43*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_0     (48U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_0, Dest signal name: bus_esm_lvl_event_48*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_1     (49U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_1, Dest signal name: bus_esm_lvl_event_49*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_2     (50U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_2, Dest signal name: bus_esm_lvl_event_50*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_3     (51U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_3, Dest signal name: bus_esm_lvl_event_51*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_4     (52U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_4, Dest signal name: bus_esm_lvl_event_52*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_5     (53U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_5, Dest signal name: bus_esm_lvl_event_53*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_6     (54U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_6, Dest signal name: bus_esm_lvl_event_54*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_7     (55U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_7, Dest signal name: bus_esm_lvl_event_55*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_8     (56U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_8, Dest signal name: bus_esm_lvl_event_56*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_9     (57U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_9, Dest signal name: bus_esm_lvl_event_57*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_10    (58U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_10, Dest signal name: bus_esm_lvl_event_58*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_11    (59U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_11, Dest signal name: bus_esm_lvl_event_59*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_12    (60U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_12, Dest signal name: bus_esm_lvl_event_60*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_13    (61U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_13, Dest signal name: bus_esm_lvl_event_61*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_14    (62U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_14, Dest signal name: bus_esm_lvl_event_62*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_15    (63U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_15, Dest signal name: bus_esm_lvl_event_63*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_16    (64U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_16, Dest signal name: bus_esm_lvl_event_64*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_17    (65U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_17, Dest signal name: bus_esm_lvl_event_65*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_18    (66U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_18, Dest signal name: bus_esm_lvl_event_66*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_19    (67U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_19, Dest signal name: bus_esm_lvl_event_67*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_20    (68U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_20, Dest signal name: bus_esm_lvl_event_68*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_21    (69U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_21, Dest signal name: bus_esm_lvl_event_69*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_22    (70U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_22, Dest signal name: bus_esm_lvl_event_70*/
#define CSL_ESM0_INTR_COMPUTE_CLUSTER0_BUS_ESM_EVENTS_OUT_LEVEL_23    (71U)  /* Design details: Source signal name: compute_cluster_maxwell_tb_vdc_main_0_bus_esm_events_out_level_23, Dest signal name: bus_esm_lvl_event_71*/
#define CSL_ESM0_INTR_PBIST0_BUS_DFT_PBIST_SAFETY_ERROR               (72U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_main_0_bus_dft_pbist_safety_error_0, Dest signal name: bus_esm_lvl_event_72*/
#define CSL_ESM0_INTR_PBIST1_BUS_DFT_PBIST_SAFETY_ERROR               (73U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_main_1_bus_dft_pbist_safety_error_0, Dest signal name: bus_esm_lvl_event_73*/
#define CSL_ESM0_INTR_SA2_UL0_BUS_SA_UL_ECC_CORR_LEVEL                (78U)  /* Design details: Source signal name: sa2_ul_main_0_bus_sa_ul_ecc_corr_level_0, Dest signal name: bus_esm_lvl_event_78*/
#define CSL_ESM0_INTR_SA2_UL0_BUS_SA_UL_ECC_UNCORR_LEVEL              (79U)  /* Design details: Source signal name: sa2_ul_main_0_bus_sa_ul_ecc_uncorr_level_0, Dest signal name: bus_esm_lvl_event_79*/
#define CSL_ESM0_INTR_PCIE0_BUS_PCIE_ECC0_CORR_LEVEL                  (80U)  /* Design details: Source signal name: pcie_g3x2_main_0_bus_pcie_ecc0_corr_level_0, Dest signal name: bus_esm_lvl_event_80*/
#define CSL_ESM0_INTR_PCIE0_BUS_PCIE_ECC0_UNCORR_LEVEL                (81U)  /* Design details: Source signal name: pcie_g3x2_main_0_bus_pcie_ecc0_uncorr_level_0, Dest signal name: bus_esm_lvl_event_81*/
#define CSL_ESM0_INTR_PCIE0_BUS_PCIE_ECC1_CORR_LEVEL                  (82U)  /* Design details: Source signal name: pcie_g3x2_main_0_bus_pcie_ecc1_corr_level_0, Dest signal name: bus_esm_lvl_event_82*/
#define CSL_ESM0_INTR_PCIE0_BUS_PCIE_ECC1_UNCORR_LEVEL                (83U)  /* Design details: Source signal name: pcie_g3x2_main_0_bus_pcie_ecc1_uncorr_level_0, Dest signal name: bus_esm_lvl_event_83*/
#define CSL_ESM0_INTR_PCIE1_BUS_PCIE_ECC0_CORR_LEVEL                  (84U)  /* Design details: Source signal name: pcie_g3x2_main_1_bus_pcie_ecc0_corr_level_0, Dest signal name: bus_esm_lvl_event_84*/
#define CSL_ESM0_INTR_PCIE1_BUS_PCIE_ECC0_UNCORR_LEVEL                (85U)  /* Design details: Source signal name: pcie_g3x2_main_1_bus_pcie_ecc0_uncorr_level_0, Dest signal name: bus_esm_lvl_event_85*/
#define CSL_ESM0_INTR_PCIE1_BUS_PCIE_ECC1_CORR_LEVEL                  (86U)  /* Design details: Source signal name: pcie_g3x2_main_1_bus_pcie_ecc1_corr_level_0, Dest signal name: bus_esm_lvl_event_86*/
#define CSL_ESM0_INTR_PCIE1_BUS_PCIE_ECC1_UNCORR_LEVEL                (87U)  /* Design details: Source signal name: pcie_g3x2_main_1_bus_pcie_ecc1_uncorr_level_0, Dest signal name: bus_esm_lvl_event_87*/
#define CSL_ESM0_INTR_MMC0_BUS_EMMCSDSS_TXMEM_CORR_ERR_LVL            (88U)  /* Design details: Source signal name: emmc4sd3ss_gs80_main_0_bus_emmcsdss_txmem_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_88*/
#define CSL_ESM0_INTR_MMC0_BUS_EMMCSDSS_TXMEM_UNCORR_ERR_LVL          (89U)  /* Design details: Source signal name: emmc4sd3ss_gs80_main_0_bus_emmcsdss_txmem_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_89*/
#define CSL_ESM0_INTR_MMC0_BUS_EMMCSDSS_RXMEM_CORR_ERR_LVL            (90U)  /* Design details: Source signal name: emmc4sd3ss_gs80_main_0_bus_emmcsdss_rxmem_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_90*/
#define CSL_ESM0_INTR_MMC0_BUS_EMMCSDSS_RXMEM_UNCORR_ERR_LVL          (91U)  /* Design details: Source signal name: emmc4sd3ss_gs80_main_0_bus_emmcsdss_rxmem_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_91*/
#define CSL_ESM0_INTR_MMC1_BUS_EMMCSDSS_TXMEM_CORR_ERR_LVL            (92U)  /* Design details: Source signal name: emmc2sd3ss_gs80_main_0_bus_emmcsdss_txmem_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_92*/
#define CSL_ESM0_INTR_MMC1_BUS_EMMCSDSS_TXMEM_UNCORR_ERR_LVL          (93U)  /* Design details: Source signal name: emmc2sd3ss_gs80_main_0_bus_emmcsdss_txmem_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_93*/
#define CSL_ESM0_INTR_MMC1_BUS_EMMCSDSS_RXMEM_CORR_ERR_LVL            (94U)  /* Design details: Source signal name: emmc2sd3ss_gs80_main_0_bus_emmcsdss_rxmem_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_94*/
#define CSL_ESM0_INTR_MMC1_BUS_EMMCSDSS_RXMEM_UNCORR_ERR_LVL          (95U)  /* Design details: Source signal name: emmc2sd3ss_gs80_main_0_bus_emmcsdss_rxmem_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_95*/
#define CSL_ESM0_INTR_USB3SS0_BUS_ECC_SEC_LVL                         (96U)  /* Design details: Source signal name: usb3ss2p0_gs80_main_0_bus_ecc_sec_lvl_0, Dest signal name: bus_esm_lvl_event_96*/
#define CSL_ESM0_INTR_USB3SS0_BUS_ECC_DED_LVL                         (97U)  /* Design details: Source signal name: usb3ss2p0_gs80_main_0_bus_ecc_ded_lvl_0, Dest signal name: bus_esm_lvl_event_97*/
#define CSL_ESM0_INTR_USB3SS1_BUS_ECC_SEC_LVL                         (98U)  /* Design details: Source signal name: usb3ss2p0_gs80_main_1_bus_ecc_sec_lvl_0, Dest signal name: bus_esm_lvl_event_98*/
#define CSL_ESM0_INTR_USB3SS1_BUS_ECC_DED_LVL                         (99U)  /* Design details: Source signal name: usb3ss2p0_gs80_main_1_bus_ecc_ded_lvl_0, Dest signal name: bus_esm_lvl_event_99*/
#define CSL_ESM0_INTR_DCC0_BUS_INTR_ERR_LEVEL                        (104U)  /* Design details: Source signal name: dcc_main_0_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_104*/
#define CSL_ESM0_INTR_DCC1_BUS_INTR_ERR_LEVEL                        (105U)  /* Design details: Source signal name: dcc_main_1_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_105*/
#define CSL_ESM0_INTR_DCC2_BUS_INTR_ERR_LEVEL                        (106U)  /* Design details: Source signal name: dcc_main_2_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_106*/
#define CSL_ESM0_INTR_DCC3_BUS_INTR_ERR_LEVEL                        (107U)  /* Design details: Source signal name: dcc_main_3_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_107*/
#define CSL_ESM0_INTR_DCC4_BUS_INTR_ERR_LEVEL                        (108U)  /* Design details: Source signal name: dcc_main_4_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_108*/
#define CSL_ESM0_INTR_DCC5_BUS_INTR_ERR_LEVEL                        (109U)  /* Design details: Source signal name: dcc_main_5_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_109*/
#define CSL_ESM0_INTR_DCC6_BUS_INTR_ERR_LEVEL                        (110U)  /* Design details: Source signal name: dcc_main_6_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_110*/
#define CSL_ESM0_INTR_DCC7_BUS_INTR_ERR_LEVEL                        (111U)  /* Design details: Source signal name: dcc_main_7_bus_intr_err_level_0, Dest signal name: bus_esm_lvl_event_111*/
#define CSL_ESM0_INTR_PDMA_DEBUG0_BUS_ECC_SEC_PEND                   (120U)  /* Design details: Source signal name: pdma_debug_main_0_bus_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_120*/
#define CSL_ESM0_INTR_PDMA_DEBUG0_BUS_ECC_DED_PEND                   (121U)  /* Design details: Source signal name: pdma_debug_main_0_bus_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_121*/
#define CSL_ESM0_INTR_PDMA0_BUS_ECC_SEC_PEND                         (122U)  /* Design details: Source signal name: pdma_main0_main_0_bus_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_122*/
#define CSL_ESM0_INTR_PDMA0_BUS_ECC_DED_PEND                         (123U)  /* Design details: Source signal name: pdma_main0_main_0_bus_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_123*/
#define CSL_ESM0_INTR_PDMA1_BUS_ECC_SEC_PEND                         (124U)  /* Design details: Source signal name: pdma_main1_main_0_bus_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_124*/
#define CSL_ESM0_INTR_PDMA1_BUS_ECC_DED_PEND                         (125U)  /* Design details: Source signal name: pdma_main1_main_0_bus_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_125*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_CTL_ECC_AGGR_CORR_ERR_LVL     (128U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_ctl_ecc_aggr_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_128*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_CTL_ECC_AGGR_UNCORR_ERR_LVL   (129U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_ctl_ecc_aggr_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_129*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_VBUS_ECC_AGGR_CORR_ERR_LVL    (130U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_vbus_ecc_aggr_corr_err_lvl_0, Dest signal name: bus_esm_lvl_event_130*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_VBUS_ECC_AGGR_UNCORR_ERR_LVL  (131U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_vbus_ecc_aggr_uncorr_err_lvl_0, Dest signal name: bus_esm_lvl_event_131*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_DRAM_ECC_CORR_ERR             (132U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_dram_ecc_corr_err_0, Dest signal name: bus_esm_lvl_event_132*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_DRAM_ECC_UNCORR_ERR           (133U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_dram_ecc_uncorr_err_0, Dest signal name: bus_esm_lvl_event_133*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_V2H_OTHER_ERR_LVL             (136U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_v2h_other_err_lvl_0, Dest signal name: bus_esm_lvl_event_136*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_DFI_ALERT_ERR_0               (137U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_dfi_alert_err_0, Dest signal name: bus_esm_lvl_event_137*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_DFI_ALERT_ERR_MAX_REACHED     (138U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_dfi_alert_err_max_reached_0, Dest signal name: bus_esm_lvl_event_138*/
#define CSL_ESM0_INTR_DDRSS0_BUS_DDRSS_DFI_ALERT_ERR_FATL            (139U)  /* Design details: Source signal name: ddr39ss_gs80_main_0_bus_ddrss_dfi_alert_err_fatl_0, Dest signal name: bus_esm_lvl_event_139*/
#define CSL_ESM0_INTR_PSRAM0_BUS_ECC_CORR_LEVEL                      (140U)  /* Design details: Source signal name: psram256x32e_main_0_bus_ecc_corr_level_0, Dest signal name: bus_esm_lvl_event_140*/
#define CSL_ESM0_INTR_PSRAM0_BUS_ECC_UNCORR_LEVEL                    (141U)  /* Design details: Source signal name: psram256x32e_main_0_bus_ecc_uncorr_level_0, Dest signal name: bus_esm_lvl_event_141*/
#define CSL_ESM0_INTR_PRU_ICSSG0_BUS_PR1_EDIO0_WD_TRIIG              (144U)  /* Design details: Source signal name: icss_g_main_0_bus_pr1_edio0_wd_triig_0, Dest signal name: bus_esm_lvl_event_144*/
#define CSL_ESM0_INTR_PRU_ICSSG0_BUS_PR1_EDIO1_WD_TRIIG              (145U)  /* Design details: Source signal name: icss_g_main_0_bus_pr1_edio1_wd_triig_0, Dest signal name: bus_esm_lvl_event_145*/
#define CSL_ESM0_INTR_PRU_ICSSG1_BUS_PR1_EDIO0_WD_TRIIG              (146U)  /* Design details: Source signal name: icss_g_main_1_bus_pr1_edio0_wd_triig_0, Dest signal name: bus_esm_lvl_event_146*/
#define CSL_ESM0_INTR_PRU_ICSSG1_BUS_PR1_EDIO1_WD_TRIIG              (147U)  /* Design details: Source signal name: icss_g_main_1_bus_pr1_edio1_wd_triig_0, Dest signal name: bus_esm_lvl_event_147*/
#define CSL_ESM0_INTR_PRU_ICSSG2_BUS_PR1_EDIO0_WD_TRIIG              (148U)  /* Design details: Source signal name: icss_g_main_2_bus_pr1_edio0_wd_triig_0, Dest signal name: bus_esm_lvl_event_148*/
#define CSL_ESM0_INTR_PRU_ICSSG2_BUS_PR1_EDIO1_WD_TRIIG              (149U)  /* Design details: Source signal name: icss_g_main_2_bus_pr1_edio1_wd_triig_0, Dest signal name: bus_esm_lvl_event_149*/
#define CSL_ESM0_INTR_PRU_ICSSG0_BUS_PR1_ECC_SEC_ERR_PEND            (150U)  /* Design details: Source signal name: icss_g_main_0_bus_pr1_ecc_sec_err_pend_0, Dest signal name: bus_esm_lvl_event_150*/
#define CSL_ESM0_INTR_PRU_ICSSG0_BUS_PR1_ECC_DED_ERR_PEND            (151U)  /* Design details: Source signal name: icss_g_main_0_bus_pr1_ecc_ded_err_pend_0, Dest signal name: bus_esm_lvl_event_151*/
#define CSL_ESM0_INTR_PRU_ICSSG1_BUS_PR1_ECC_SEC_ERR_PEND            (152U)  /* Design details: Source signal name: icss_g_main_1_bus_pr1_ecc_sec_err_pend_0, Dest signal name: bus_esm_lvl_event_152*/
#define CSL_ESM0_INTR_PRU_ICSSG1_BUS_PR1_ECC_DED_ERR_PEND            (153U)  /* Design details: Source signal name: icss_g_main_1_bus_pr1_ecc_ded_err_pend_0, Dest signal name: bus_esm_lvl_event_153*/
#define CSL_ESM0_INTR_PRU_ICSSG2_BUS_PR1_ECC_SEC_ERR_PEND            (154U)  /* Design details: Source signal name: icss_g_main_2_bus_pr1_ecc_sec_err_pend_0, Dest signal name: bus_esm_lvl_event_154*/
#define CSL_ESM0_INTR_PRU_ICSSG2_BUS_PR1_ECC_DED_ERR_PEND            (155U)  /* Design details: Source signal name: icss_g_main_2_bus_pr1_ecc_ded_err_pend_0, Dest signal name: bus_esm_lvl_event_155*/
#define CSL_ESM0_INTR_NAVSS0_BUS_MODSS_ECC_SEC_PEND                  (160U)  /* Design details: Source signal name: navss256l_main_0_bus_modss_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_160*/
#define CSL_ESM0_INTR_NAVSS0_BUS_MODSS_ECC_DED_PEND                  (161U)  /* Design details: Source signal name: navss256l_main_0_bus_modss_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_161*/
#define CSL_ESM0_INTR_NAVSS0_BUS_UDMASS_ECC_SEC_PEND                 (162U)  /* Design details: Source signal name: navss256l_main_0_bus_udmass_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_162*/
#define CSL_ESM0_INTR_NAVSS0_BUS_UDMASS_ECC_DED_PEND                 (163U)  /* Design details: Source signal name: navss256l_main_0_bus_udmass_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_163*/
#define CSL_ESM0_INTR_NAVSS0_BUS_NBSS_ECC_SEC_PEND                   (164U)  /* Design details: Source signal name: navss256l_main_0_bus_nbss_ecc_sec_pend_0, Dest signal name: bus_esm_lvl_event_164*/
#define CSL_ESM0_INTR_NAVSS0_BUS_NBSS_ECC_DED_PEND                   (165U)  /* Design details: Source signal name: navss256l_main_0_bus_nbss_ecc_ded_pend_0, Dest signal name: bus_esm_lvl_event_165*/
#define CSL_ESM0_INTR_ECC_AGGR1_BUS_CORR_LEVEL                       (168U)  /* Design details: Source signal name: m4_mainclk2_ecc_aggr_main_0_bus_corr_level_0, Dest signal name: bus_esm_lvl_event_168*/
#define CSL_ESM0_INTR_ECC_AGGR1_BUS_UNCORR_LEVEL                     (169U)  /* Design details: Source signal name: m4_mainclk2_ecc_aggr_main_0_bus_uncorr_level_0, Dest signal name: bus_esm_lvl_event_169*/
#define CSL_ESM0_INTR_ECC_AGGR2_BUS_CORR_LEVEL                       (170U)  /* Design details: Source signal name: m4_mainclk4_ecc_aggr_main_0_bus_corr_level_0, Dest signal name: bus_esm_lvl_event_170*/
#define CSL_ESM0_INTR_ECC_AGGR2_BUS_UNCORR_LEVEL                     (171U)  /* Design details: Source signal name: m4_mainclk4_ecc_aggr_main_0_bus_uncorr_level_0, Dest signal name: bus_esm_lvl_event_171*/
#define CSL_ESM0_INTR_ECC_AGGR0_BUS_UNCORR_LEVEL                     (172U)  /* Design details: Source signal name: m4_main_infraclk4_ecc_aggr_main_0_bus_uncorr_level_0, Dest signal name: bus_esm_lvl_event_172*/
#define CSL_ESM0_INTR_ECC_AGGR0_BUS_CORR_LEVEL                       (173U)  /* Design details: Source signal name: m4_main_infraclk4_ecc_aggr_main_0_bus_corr_level_0, Dest signal name: bus_esm_lvl_event_173*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MAIN_INFRA_CS1_CLKSTOP_REQ    (176U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_main_infra_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_176*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MAIN_TEST_CS1_CLKSTOP_REQ     (177U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_main_test_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_177*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MAIN_PBIST_CS1_CLKSTOP_REQ    (178U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_main_pbist_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_178*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MAIN_RESERVE_CS1_CLKSTOP_REQ  (179U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_main_reserve_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_179*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_CC_RESERVE_CS1_CLKSTOP_REQ    (180U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_cc_reserve_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_180*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_CC_TOP_CS1_CLKSTOP_REQ   (181U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_cc_top_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_181*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_CC_TOP_PBIST_CS1_CLKSTOP_REQ  (182U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_cc_top_pbist_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_182*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_0_CS1_CLKSTOP_REQ    (183U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_183*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_1_CS1_CLKSTOP_REQ    (184U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_184*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_2_CS1_CLKSTOP_REQ    (185U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_2_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_185*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_3_CS1_CLKSTOP_REQ    (186U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_3_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_186*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_CL_0_CS1_CLKSTOP_REQ (187U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_cl_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_187*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_CL_0_PBIST_CS1_CLKSTOP_REQ     (188U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_cl_0_pbist_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_188*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_CL_1_CS1_CLKSTOP_REQ (189U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_cl_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_189*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_A53_CL_1_PBIST_CS1_CLKSTOP_REQ     (190U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_a53_cl_1_pbist_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_190*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MAIN_DEBUG_CS1_CLKSTOP_REQ    (191U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_main_debug_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_191*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_DSS_CS1_CLKSTOP_REQ      (192U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_dss_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_192*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_MMC_CS1_CLKSTOP_REQ      (193U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_mmc_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_193*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_CAL_CS1_CLKSTOP_REQ      (194U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_cal_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_194*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_PCIE_0_CS1_CLKSTOP_REQ   (195U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_pcie_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_195*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_PCIE_1_CS1_CLKSTOP_REQ   (196U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_pcie_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_196*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_USB_0_CS1_CLKSTOP_REQ    (197U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_usb_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_197*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_USB_1_CS1_CLKSTOP_REQ    (198U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_usb_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_198*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_SAUL_CS1_CLKSTOP_REQ     (199U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_saul_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_199*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_PER_COMMON_CS1_CLKSTOP_REQ    (200U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_per_common_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_200*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_NB_CS1_CLKSTOP_REQ       (201U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_nb_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_201*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_PER_RESERVE_CS1_CLKSTOP_REQ   (202U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_per_reserve_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_202*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_SERDES_0_CS1_CLKSTOP_REQ (203U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_serdes_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_203*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_SERDES_1_CS1_CLKSTOP_REQ (204U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_serdes_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_204*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_ICSSG_0_CS1_CLKSTOP_REQ  (205U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_icssg_0_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_205*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_ICSSG_1_CS1_CLKSTOP_REQ  (206U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_icssg_1_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_206*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_ICSSG_2_CS1_CLKSTOP_REQ  (207U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_icssg_2_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_207*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_GPU_CS1_CLKSTOP_REQ      (208U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_gpu_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_208*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_GPU_PBIST_CS1_CLKSTOP_REQ     (209U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_gpu_pbist_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_209*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_EMIF_DATA_CS1_CLKSTOP_REQ     (210U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_emif_data_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_210*/
#define CSL_ESM0_INTR_PSC0_BUS_PSC_MOD_MNLP_EMIF_CFG_CS1_CLKSTOP_REQ (211U)  /* Design details: Source signal name: k3_main_psc_wrap_main_0_bus_psc_mod_mnlp_emif_cfg_cs1_clkstop_req_0, Dest signal name: bus_esm_lvl_event_211*/
#define CSL_ESM0_INTR_RTI0_BUS_INTR_WWD                              (224U)  /* Design details: Source signal name: rti_main_0_bus_intr_wwd_0, Dest signal name: bus_esm_pls_event_0*/
#define CSL_ESM0_INTR_RTI1_BUS_INTR_WWD                              (225U)  /* Design details: Source signal name: rti_main_1_bus_intr_wwd_0, Dest signal name: bus_esm_pls_event_1*/
#define CSL_ESM0_INTR_RTI2_BUS_INTR_WWD                              (226U)  /* Design details: Source signal name: rti_main_2_bus_intr_wwd_0, Dest signal name: bus_esm_pls_event_2*/
#define CSL_ESM0_INTR_RTI3_BUS_INTR_WWD                              (227U)  /* Design details: Source signal name: rti_main_3_bus_intr_wwd_0, Dest signal name: bus_esm_pls_event_3*/
#define CSL_ESM0_INTR_GIC0_BUS_AXIM_ERR                              (240U)  /* Design details: Source signal name: gic500ss_main_0_bus_axim_err_0, Dest signal name: bus_esm_pls_event0_240*/
#define CSL_ESM0_INTR_GIC0_BUS_ECC_FATAL                             (241U)  /* Design details: Source signal name: gic500ss_main_0_bus_ecc_fatal_0, Dest signal name: bus_esm_pls_event0_241*/
#define CSL_ESM0_INTR_GIC0_BUS_ECC_AGGR_UNCORR_PULSE                 (242U)  /* Design details: Source signal name: gic500ss_main_0_bus_ecc_aggr_uncorr_pulse_0, Dest signal name: bus_esm_pls_event0_242*/
#define CSL_ESM0_INTR_GIC0_BUS_ECC_AGGR_CORR_PULSE                   (243U)  /* Design details: Source signal name: gic500ss_main_0_bus_ecc_aggr_corr_pulse_0, Dest signal name: bus_esm_pls_event0_243*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_0    (248U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_0, Dest signal name: bus_esm_pls_event0_248*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_1    (249U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_1, Dest signal name: bus_esm_pls_event0_249*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_2    (250U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_2, Dest signal name: bus_esm_pls_event0_250*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_3    (251U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_3, Dest signal name: bus_esm_pls_event0_251*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_4    (252U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_4, Dest signal name: bus_esm_pls_event0_252*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_5    (253U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_5, Dest signal name: bus_esm_pls_event0_253*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_6    (254U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_6, Dest signal name: bus_esm_pls_event0_254*/
#define CSL_ESM0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_7    (255U)  /* Design details: Source signal name: main_gpiomux_introuter_main_0_bus_outp_7, Dest signal name: bus_esm_pls_event0_255*/

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_ESM0_H_*/
