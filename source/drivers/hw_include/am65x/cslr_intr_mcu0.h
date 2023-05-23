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
#ifndef CSLR_INTR_MCU0_H_
#define CSLR_INTR_MCU0_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* List of intr sources for receiver: mcu0 */
/* This instance name corresponds to design instance name: pulsar_sl_mcu_0 */
#define CSL_MCU0_INTR_MCAN0_MCANSS_MCAN_LVL_INT_0                      (0U)  /* Design details: Source signal name: mcanss_mcu_0_mcanss_mcan_lvl_int_0, Dest signal name: cpu0_intr_0 */
#define CSL_MCU0_INTR_MCAN0_MCANSS_MCAN_LVL_INT_1                      (1U)  /* Design details: Source signal name: mcanss_mcu_0_mcanss_mcan_lvl_int_1, Dest signal name: cpu0_intr_1 */
#define CSL_MCU0_INTR_MCAN1_MCANSS_MCAN_LVL_INT_0                      (2U)  /* Design details: Source signal name: mcanss_mcu_1_mcanss_mcan_lvl_int_0, Dest signal name: cpu0_intr_2 */
#define CSL_MCU0_INTR_MCAN1_MCANSS_MCAN_LVL_INT_1                      (3U)  /* Design details: Source signal name: mcanss_mcu_1_mcanss_mcan_lvl_int_1, Dest signal name: cpu0_intr_3 */
#define CSL_MCU0_INTR_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT             (4U)  /* Design details: Source signal name: mcanss_mcu_0_mcanss_ext_ts_rollover_lvl_int_0, Dest signal name: cpu0_intr_4 */
#define CSL_MCU0_INTR_MCAN1_MCANSS_EXT_TS_ROLLOVER_LVL_INT             (5U)  /* Design details: Source signal name: mcanss_mcu_1_mcanss_ext_ts_rollover_lvl_int_0, Dest signal name: cpu0_intr_5 */
#define CSL_MCU0_INTR_ADC0_GEN_LEVEL                                   (6U)  /* Design details: Source signal name: adc12_gs80_mcu_0_gen_level_0, Dest signal name: cpu0_intr_6 */
#define CSL_MCU0_INTR_ADC1_GEN_LEVEL                                   (7U)  /* Design details: Source signal name: adc12_gs80_mcu_1_gen_level_0, Dest signal name: cpu0_intr_7 */
#define CSL_MCU0_INTR_MCSPI0_INTR_SPI                                 (20U)  /* Design details: Source signal name: spi_mcu_0_intr_spi_0, Dest signal name: cpu0_intr_20 */
#define CSL_MCU0_INTR_MCSPI1_INTR_SPI                                 (21U)  /* Design details: Source signal name: spi_mcu_1_intr_spi_0, Dest signal name: cpu0_intr_21 */
#define CSL_MCU0_INTR_MCSPI2_INTR_SPI                                 (22U)  /* Design details: Source signal name: spi_mcu_2_intr_spi_0, Dest signal name: cpu0_intr_22 */
#define CSL_MCU0_INTR_I2C1_POINTRPEND                                 (23U)  /* Design details: Source signal name: mshsi2c_mcu_0_pointrpend_0, Dest signal name: cpu0_intr_23 */
#define CSL_MCU0_INTR_FSS0_OSPI0_LVL_INTR                             (24U)  /* Design details: Source signal name: fss_mcu_0_ospi0_lvl_intr_0, Dest signal name: cpu0_intr_24 */
#define CSL_MCU0_INTR_FSS0_OSPI1_LVL_INTR                             (25U)  /* Design details: Source signal name: fss_mcu_0_ospi1_lvl_intr_0, Dest signal name: cpu0_intr_25 */
#define CSL_MCU0_INTR_FSS0_HPB_INTR                                   (26U)  /* Design details: Source signal name: fss_mcu_0_hpb_intr_0, Dest signal name: cpu0_intr_26 */
#define CSL_MCU0_INTR_FSS0_OTFA_INTR_ERR_PEND                         (27U)  /* Design details: Source signal name: fss_mcu_0_otfa_intr_err_pend_0, Dest signal name: cpu0_intr_27 */
#define CSL_MCU0_INTR_FSS0_FSAS_ECC_INTR_ERR_PEND                     (28U)  /* Design details: Source signal name: fss_mcu_0_fsas_ecc_intr_err_pend_0, Dest signal name: cpu0_intr_28 */
#define CSL_MCU0_INTR_UART1_USART_IRQ                                 (30U)  /* Design details: Source signal name: usart_mcu_0_usart_irq_0, Dest signal name: cpu0_intr_30 */
#define CSL_MCU0_INTR_CPSW0_STAT_PEND                                 (32U)  /* Design details: Source signal name: cpsw_2guss_mcu_0_stat_pend_0, Dest signal name: cpu0_intr_32 */
#define CSL_MCU0_INTR_CPSW0_EVNT_PEND                                 (34U)  /* Design details: Source signal name: cpsw_2guss_mcu_0_evnt_pend_0, Dest signal name: cpu0_intr_34 */
#define CSL_MCU0_INTR_CPSW0_MDIO_PEND                                 (35U)  /* Design details: Source signal name: cpsw_2guss_mcu_0_mdio_pend_0, Dest signal name: cpu0_intr_35 */
#define CSL_MCU0_INTR_TIMER0_INTR_PEND                                (38U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_0_intr_pend_0, Dest signal name: cpu0_intr_38 */
#define CSL_MCU0_INTR_TIMER1_INTR_PEND                                (39U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_1_intr_pend_0, Dest signal name: cpu0_intr_39 */
#define CSL_MCU0_INTR_TIMER2_INTR_PEND                                (40U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_2_intr_pend_0, Dest signal name: cpu0_intr_40 */
#define CSL_MCU0_INTR_TIMER3_INTR_PEND                                (41U)  /* Design details: Source signal name: dmtimer_dmc1ms_mcu_3_intr_pend_0, Dest signal name: cpu0_intr_41 */
#define CSL_MCU0_INTR_RTI0_INTR_WWD                                   (42U)  /* Design details: Source signal name: rti_mcu_0_intr_wwd_0, Dest signal name: cpu0_intr_42 */
#define CSL_MCU0_INTR_RTI1_INTR_WWD                                   (43U)  /* Design details: Source signal name: rti_mcu_1_intr_wwd_0, Dest signal name: cpu0_intr_43 */
#define CSL_MCU0_INTR_MCU_CTRL0_IPC_SET0_IPC_SET_IPCFG                (44U)  /* Design details: Source signal name: mcu_ctrl_mmr_mcu_0_IPC_SET0_ipc_set_ipcfg_0, Dest signal name: cpu0_intr_44 */
#define CSL_MCU0_INTR_MCU_CTRL0_IPC_SET1_IPC_SET_IPCFG                (45U)  /* Design details: Source signal name: mcu_ctrl_mmr_mcu_0_IPC_SET1_ipc_set_ipcfg_0, Dest signal name: cpu0_intr_45 */
#define CSL_MCU0_INTR_MCU_CTRL0_ACCESS_ERR                            (46U)  /* Design details: Source signal name: mcu_ctrl_mmr_mcu_0_access_err_0, Dest signal name: cpu0_intr_46 */
#define CSL_MCU0_INTR_PBIST0_DFT_PBIST_CPU                            (47U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_mcu_0_dft_pbist_cpu_0, Dest signal name: cpu0_intr_47 */
#define CSL_MCU0_INTR_ESM1_ESM_INT_LOW_LVL                            (48U)  /* Design details: Source signal name: esm_mcu_mcu_0_esm_int_low_lvl_0, Dest signal name: cpu0_intr_48 */
#define CSL_MCU0_INTR_ESM1_ESM_INT_HI_LVL                             (49U)  /* Design details: Source signal name: esm_mcu_mcu_0_esm_int_hi_lvl_0, Dest signal name: cpu0_intr_49 */
#define CSL_MCU0_INTR_DCC0_INTR_DONE_LEVEL                            (50U)  /* Design details: Source signal name: dcc_mcu_0_intr_done_level_0, Dest signal name: cpu0_intr_50 */
#define CSL_MCU0_INTR_DCC1_INTR_DONE_LEVEL                            (51U)  /* Design details: Source signal name: dcc_mcu_1_intr_done_level_0, Dest signal name: cpu0_intr_51 */
#define CSL_MCU0_INTR_DCC2_INTR_DONE_LEVEL                            (52U)  /* Design details: Source signal name: dcc_mcu_2_intr_done_level_0, Dest signal name: cpu0_intr_52 */
#define CSL_MCU0_INTR_ESM1_ESM_INT_CFG_LVL                            (53U)  /* Design details: Source signal name: esm_mcu_mcu_0_esm_int_cfg_lvl_0, Dest signal name: cpu0_intr_53 */
#define CSL_MCU0_INTR_MCUDBG0_AQCMPINTR_LEVEL                         (54U)  /* Design details: Source signal name: k3_mcu_debug_cell_main_0_aqcmpintr_level_0, Dest signal name: cpu0_intr_54 */
#define CSL_MCU0_INTR_MCUDBG0_CTM_LEVEL                               (55U)  /* Design details: Source signal name: k3_mcu_debug_cell_main_0_ctm_level_0, Dest signal name: cpu0_intr_55 */
#define CSL_MCU0_INTR_MCU0_CPU0_VALIRQ                                (56U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_valirq_0, Dest signal name: cpu0_intr_56 */
#define CSL_MCU0_INTR_MCU0_CPU0_VALFIQ                                (57U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_valfiq_0, Dest signal name: cpu0_intr_57 */
#define CSL_MCU0_INTR_MCU0_CPU0_CTI                                   (58U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu0_cti_0, Dest signal name: cpu0_intr_58 */
#define CSL_MCU0_INTR_MCU0_CPU1_CTI                                   (59U)  /* Design details: Source signal name: pulsar_sl_mcu_0_cpu1_cti_0, Dest signal name: cpu0_intr_59 */
#define CSL_MCU0_INTR_MCU0_COMMRX_LEVEL_0                             (60U)  /* Design details: Source signal name: pulsar_sl_mcu_0_commrx_level_0_0, Dest signal name: cpu0_intr_60 */
#define CSL_MCU0_INTR_MCU0_COMMTX_LEVEL_0                             (61U)  /* Design details: Source signal name: pulsar_sl_mcu_0_commtx_level_0_0, Dest signal name: cpu0_intr_61 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_0                              (64U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_0, Dest signal name: cpu0_intr_64 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_1                              (65U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_1, Dest signal name: cpu0_intr_65 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_2                              (66U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_2, Dest signal name: cpu0_intr_66 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_3                              (67U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_3, Dest signal name: cpu0_intr_67 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_4                              (68U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_4, Dest signal name: cpu0_intr_68 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_5                              (69U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_5, Dest signal name: cpu0_intr_69 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_6                              (70U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_6, Dest signal name: cpu0_intr_70 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_7                              (71U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_7, Dest signal name: cpu0_intr_71 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_8                              (72U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_8, Dest signal name: cpu0_intr_72 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_9                              (73U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_9, Dest signal name: cpu0_intr_73 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_10                             (74U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_10, Dest signal name: cpu0_intr_74 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_11                             (75U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_11, Dest signal name: cpu0_intr_75 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_12                             (76U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_12, Dest signal name: cpu0_intr_76 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_13                             (77U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_13, Dest signal name: cpu0_intr_77 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_14                             (78U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_14, Dest signal name: cpu0_intr_78 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_15                             (79U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_15, Dest signal name: cpu0_intr_79 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_16                             (80U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_16, Dest signal name: cpu0_intr_80 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_17                             (81U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_17, Dest signal name: cpu0_intr_81 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_18                             (82U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_18, Dest signal name: cpu0_intr_82 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_19                             (83U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_19, Dest signal name: cpu0_intr_83 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_20                             (84U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_20, Dest signal name: cpu0_intr_84 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_21                             (85U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_21, Dest signal name: cpu0_intr_85 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_22                             (86U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_22, Dest signal name: cpu0_intr_86 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_23                             (87U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_23, Dest signal name: cpu0_intr_87 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_24                             (88U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_24, Dest signal name: cpu0_intr_88 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_25                             (89U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_25, Dest signal name: cpu0_intr_89 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_26                             (90U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_26, Dest signal name: cpu0_intr_90 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_27                             (91U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_27, Dest signal name: cpu0_intr_91 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_28                             (92U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_28, Dest signal name: cpu0_intr_92 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_29                             (93U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_29, Dest signal name: cpu0_intr_93 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_30                             (94U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_30, Dest signal name: cpu0_intr_94 */
#define CSL_MCU0_INTR_NAVSS0_R5_0_PEND_31                             (95U)  /* Design details: Source signal name: navss_mcu_mcu_0_r5_0_pend_31, Dest signal name: cpu0_intr_95 */
#define CSL_MCU0_INTR_I2C0_POINTRPEND                                 (96U)  /* Design details: Source signal name: mshsi2c_wkup_0_pointrpend_0, Dest signal name: cpu0_intr_96 */
#define CSL_MCU0_INTR_UART0_USART_IRQ                                 (97U)  /* Design details: Source signal name: usart_wkup_0_usart_irq_0, Dest signal name: cpu0_intr_97 */
#define CSL_MCU0_INTR_ESM0_ESM_INT_LOW_LVL                            (98U)  /* Design details: Source signal name: esm_wkup_wkup_0_esm_int_low_lvl_0, Dest signal name: cpu0_intr_98 */
#define CSL_MCU0_INTR_ESM0_ESM_INT_HI_LVL                             (99U)  /* Design details: Source signal name: esm_wkup_wkup_0_esm_int_hi_lvl_0, Dest signal name: cpu0_intr_99 */
#define CSL_MCU0_INTR_ESM0_ESM_INT_CFG_LVL                           (100U)  /* Design details: Source signal name: esm_wkup_wkup_0_esm_int_cfg_lvl_0, Dest signal name: cpu0_intr_100 */
#define CSL_MCU0_INTR_DMSC0_DBG_AUTH                                 (103U)  /* Design details: Source signal name: dmsc_wkup_0_dbg_auth_0, Dest signal name: cpu0_intr_103 */
#define CSL_MCU0_INTR_DMSC0_AES_SINTREQUEST_P                        (104U)  /* Design details: Source signal name: dmsc_wkup_0_aes_sintrequest_p_0, Dest signal name: cpu0_intr_104 */
#define CSL_MCU0_INTR_DMSC0_AES_SINTREQUEST_S                        (105U)  /* Design details: Source signal name: dmsc_wkup_0_aes_sintrequest_s_0, Dest signal name: cpu0_intr_105 */
#define CSL_MCU0_INTR_DMSC0_SEC_OUT_0                                (106U)  /* Design details: Source signal name: dmsc_wkup_0_sec_out_0, Dest signal name: cpu0_intr_106 */
#define CSL_MCU0_INTR_DMSC0_SEC_OUT_1                                (107U)  /* Design details: Source signal name: dmsc_wkup_0_sec_out_1, Dest signal name: cpu0_intr_107 */
#define CSL_MCU0_INTR_WKUP_CTRL_MMR0_ACCESS_ERR                      (119U)  /* Design details: Source signal name: wkup_ctrl_mmr_wkup_0_access_err_0, Dest signal name: cpu0_intr_119 */
#define CSL_MCU0_INTR_GLUE_SOC_PORZ_LATCHED_GLUE_SOC_PORZ_LATCHED_SOC_PORZ_LATCHED  (120U)  /* Design details: Source signal name: GLUE_soc_porz_latched_soc_porz_latched_0, Dest signal name: cpu0_intr_120 */
#define CSL_MCU0_INTR_GLUE_SOCPRG_PRZ_LATCHED_GLUE_SOCPRG_PRZ_LATCHED_SOCPRG_PRZ_LATCHED (121U)  /* Design details: Source signal name: GLUE_socprg_prz_latched_socprg_prz_latched_0, Dest signal name: cpu0_intr_121 */
#define CSL_MCU0_INTR_GLUE_SOC_RESETZ_LATCHED_GLUE_SOC_RESETZ_LATCHED_SOC_RESETZ_LATCHED (122U)  /* Design details: Source signal name: GLUE_soc_resetz_latched_soc_resetz_latched_0, Dest signal name: cpu0_intr_122 */
#define CSL_MCU0_INTR_GLUE_MCU_RESETZ_LATCHED_GLUE_MCU_RESETZ_LATCHED_MCU_RESETZ_LATCHED (123U)  /* Design details: Source signal name: GLUE_mcu_resetz_latched_mcu_resetz_latched_0, Dest signal name: cpu0_intr_123 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_0                           (124U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_0, Dest signal name: cpu0_intr_124 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_1                           (125U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_1, Dest signal name: cpu0_intr_125 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_2                           (126U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_2, Dest signal name: cpu0_intr_126 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_3                           (127U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_3, Dest signal name: cpu0_intr_127 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_4                           (128U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_4, Dest signal name: cpu0_intr_128 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_5                           (129U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_5, Dest signal name: cpu0_intr_129 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_6                           (130U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_6, Dest signal name: cpu0_intr_130 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_7                           (131U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_7, Dest signal name: cpu0_intr_131 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_8                           (132U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_8, Dest signal name: cpu0_intr_132 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_9                           (133U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_9, Dest signal name: cpu0_intr_133 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_10                          (134U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_10, Dest signal name: cpu0_intr_134 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_11                          (135U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_11, Dest signal name: cpu0_intr_135 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_12                          (136U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_12, Dest signal name: cpu0_intr_136 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_13                          (137U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_13, Dest signal name: cpu0_intr_137 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_14                          (138U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_14, Dest signal name: cpu0_intr_138 */
#define CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_15                          (139U)  /* Design details: Source signal name: wkup_gpiomux_introuter_wkup_0_outp_15, Dest signal name: cpu0_intr_139 */
#define CSL_MCU0_INTR_ESM2_ESM_INT_LOW_LVL                           (140U)  /* Design details: Source signal name: esm_main_main_0_esm_int_low_lvl_0, Dest signal name: cpu0_intr_140 */
#define CSL_MCU0_INTR_ESM2_ESM_INT_HI_LVL                            (141U)  /* Design details: Source signal name: esm_main_main_0_esm_int_hi_lvl_0, Dest signal name: cpu0_intr_141 */
#define CSL_MCU0_INTR_ESM2_ESM_INT_CFG_LVL                           (142U)  /* Design details: Source signal name: esm_main_main_0_esm_int_cfg_lvl_0, Dest signal name: cpu0_intr_142 */
#define CSL_MCU0_INTR_PBIST1_DFT_PBIST_CPU                           (146U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_main_0_dft_pbist_cpu_0, Dest signal name: cpu0_intr_146 */
#define CSL_MCU0_INTR_PBIST2_DFT_PBIST_CPU                           (147U)  /* Design details: Source signal name: k3_pbist_4c28p_wrap_main_1_dft_pbist_cpu_0, Dest signal name: cpu0_intr_147 */
#define CSL_MCU0_INTR_CBASS1_LPSC_MCU_COMMON_ERR_INTR_0              (148U)  /* Design details: Source signal name: m4_mcu_cbass_mcu_0_LPSC_mcu_common_err_intr_0, Dest signal name: cpu0_intr_148 */
#define CSL_MCU0_INTR_M4_MCU_DBG_CBASS_M4_MCU_DBG_CBASS_MCU_0_LPSC_MCU_DEBUG_ERR_INTR_0  (149U)  /* Design details: Source signal name: m4_mcu_dbg_cbass_mcu_0_LPSC_MCU_Debug_err_intr_0, Dest signal name: cpu0_intr_149 */
#define CSL_MCU0_INTR_M4_MCU_FW_CBASS_M4_MCU_FW_CBASS_MCU_0_LPSC_MCU_COMMON_ERR_INTR_0   (150U)  /* Design details: Source signal name: m4_mcu_fw_cbass_mcu_0_LPSC_MCU_common_err_intr_0, Dest signal name: cpu0_intr_150 */
#define CSL_MCU0_INTR_CBASS0_LPSC_WKUP_COMMON_ERR_INTR_0             (151U)  /* Design details: Source signal name: m4_wkup_cbass_wkup_0_LPSC_wkup_common_err_intr_0, Dest signal name: cpu0_intr_151 */
#define CSL_MCU0_INTR_M4_WKUP_FW_CBASS_M4_WKUP_FW_CBASS_WKUP_0_LPSC_WKUP_COMMON_ERR_INTR_0    (152U)  /* Design details: Source signal name: m4_wkup_fw_cbass_wkup_0_LPSC_wkup_common_err_intr_0, Dest signal name: cpu0_intr_152 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_0                      (160U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_0, Dest signal name: cpu0_intr_160 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_1                      (161U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_1, Dest signal name: cpu0_intr_161 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_2                      (162U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_2, Dest signal name: cpu0_intr_162 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_3                      (163U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_3, Dest signal name: cpu0_intr_163 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_4                      (164U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_4, Dest signal name: cpu0_intr_164 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_5                      (165U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_5, Dest signal name: cpu0_intr_165 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_6                      (166U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_6, Dest signal name: cpu0_intr_166 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_7                      (167U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_7, Dest signal name: cpu0_intr_167 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_8                      (168U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_8, Dest signal name: cpu0_intr_168 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_9                      (169U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_9, Dest signal name: cpu0_intr_169 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_10                     (170U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_10, Dest signal name: cpu0_intr_170 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_11                     (171U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_11, Dest signal name: cpu0_intr_171 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_12                     (172U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_12, Dest signal name: cpu0_intr_172 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_13                     (173U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_13, Dest signal name: cpu0_intr_173 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_14                     (174U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_14, Dest signal name: cpu0_intr_174 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_15                     (175U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_15, Dest signal name: cpu0_intr_175 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_16                     (176U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_16, Dest signal name: cpu0_intr_176 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_17                     (177U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_17, Dest signal name: cpu0_intr_177 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_18                     (178U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_18, Dest signal name: cpu0_intr_178 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_19                     (179U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_19, Dest signal name: cpu0_intr_179 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_20                     (180U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_20, Dest signal name: cpu0_intr_180 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_21                     (181U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_21, Dest signal name: cpu0_intr_181 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_22                     (182U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_22, Dest signal name: cpu0_intr_182 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_23                     (183U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_23, Dest signal name: cpu0_intr_183 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_24                     (184U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_24, Dest signal name: cpu0_intr_184 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_25                     (185U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_25, Dest signal name: cpu0_intr_185 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_26                     (186U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_26, Dest signal name: cpu0_intr_186 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_27                     (187U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_27, Dest signal name: cpu0_intr_187 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_28                     (188U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_28, Dest signal name: cpu0_intr_188 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_29                     (189U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_29, Dest signal name: cpu0_intr_189 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_30                     (190U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_30, Dest signal name: cpu0_intr_190 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_31                     (191U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_31, Dest signal name: cpu0_intr_191 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_32                     (192U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_32, Dest signal name: cpu0_intr_192 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_33                     (193U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_33, Dest signal name: cpu0_intr_193 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_34                     (194U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_34, Dest signal name: cpu0_intr_194 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_35                     (195U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_35, Dest signal name: cpu0_intr_195 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_36                     (196U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_36, Dest signal name: cpu0_intr_196 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_37                     (197U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_37, Dest signal name: cpu0_intr_197 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_38                     (198U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_38, Dest signal name: cpu0_intr_198 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_39                     (199U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_39, Dest signal name: cpu0_intr_199 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_40                     (200U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_40, Dest signal name: cpu0_intr_200 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_41                     (201U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_41, Dest signal name: cpu0_intr_201 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_42                     (202U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_42, Dest signal name: cpu0_intr_202 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_43                     (203U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_43, Dest signal name: cpu0_intr_203 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_44                     (204U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_44, Dest signal name: cpu0_intr_204 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_45                     (205U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_45, Dest signal name: cpu0_intr_205 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_46                     (206U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_46, Dest signal name: cpu0_intr_206 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_47                     (207U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_47, Dest signal name: cpu0_intr_207 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_48                     (208U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_48, Dest signal name: cpu0_intr_208 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_49                     (209U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_49, Dest signal name: cpu0_intr_209 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_50                     (210U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_50, Dest signal name: cpu0_intr_210 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_51                     (211U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_51, Dest signal name: cpu0_intr_211 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_52                     (212U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_52, Dest signal name: cpu0_intr_212 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_53                     (213U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_53, Dest signal name: cpu0_intr_213 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_54                     (214U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_54, Dest signal name: cpu0_intr_214 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_55                     (215U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_55, Dest signal name: cpu0_intr_215 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_56                     (216U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_56, Dest signal name: cpu0_intr_216 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_57                     (217U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_57, Dest signal name: cpu0_intr_217 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_58                     (218U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_58, Dest signal name: cpu0_intr_218 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_59                     (219U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_59, Dest signal name: cpu0_intr_219 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_60                     (220U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_60, Dest signal name: cpu0_intr_220 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_61                     (221U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_61, Dest signal name: cpu0_intr_221 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_62                     (222U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_62, Dest signal name: cpu0_intr_222 */
#define CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_63                     (223U)  /* Design details: Source signal name: main2mcu_lvl_introuter_main_0_outl_63, Dest signal name: cpu0_intr_223 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_0                    (224U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_0, Dest signal name: cpu0_intr_224 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_1                    (225U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_1, Dest signal name: cpu0_intr_225 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_2                    (226U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_2, Dest signal name: cpu0_intr_226 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_3                    (227U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_3, Dest signal name: cpu0_intr_227 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_4                    (228U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_4, Dest signal name: cpu0_intr_228 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_5                    (229U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_5, Dest signal name: cpu0_intr_229 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_6                    (230U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_6, Dest signal name: cpu0_intr_230 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_7                    (231U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_7, Dest signal name: cpu0_intr_231 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_8                    (232U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_8, Dest signal name: cpu0_intr_232 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_9                    (233U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_9, Dest signal name: cpu0_intr_233 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_10                   (234U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_10, Dest signal name: cpu0_intr_234 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_11                   (235U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_11, Dest signal name: cpu0_intr_235 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_12                   (236U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_12, Dest signal name: cpu0_intr_236 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_13                   (237U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_13, Dest signal name: cpu0_intr_237 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_14                   (238U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_14, Dest signal name: cpu0_intr_238 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_15                   (239U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_15, Dest signal name: cpu0_intr_239 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_16                   (240U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_16, Dest signal name: cpu0_intr_240 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_17                   (241U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_17, Dest signal name: cpu0_intr_241 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_18                   (242U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_18, Dest signal name: cpu0_intr_242 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_19                   (243U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_19, Dest signal name: cpu0_intr_243 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_20                   (244U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_20, Dest signal name: cpu0_intr_244 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_21                   (245U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_21, Dest signal name: cpu0_intr_245 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_22                   (246U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_22, Dest signal name: cpu0_intr_246 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_23                   (247U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_23, Dest signal name: cpu0_intr_247 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_24                   (248U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_24, Dest signal name: cpu0_intr_248 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_25                   (249U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_25, Dest signal name: cpu0_intr_249 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_26                   (250U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_26, Dest signal name: cpu0_intr_250 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_27                   (251U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_27, Dest signal name: cpu0_intr_251 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_28                   (252U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_28, Dest signal name: cpu0_intr_252 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_29                   (253U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_29, Dest signal name: cpu0_intr_253 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_30                   (254U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_30, Dest signal name: cpu0_intr_254 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_31                   (255U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_31, Dest signal name: cpu0_intr_255 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_32                   (256U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_32, Dest signal name: cpu0_intr_256 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_33                   (257U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_33, Dest signal name: cpu0_intr_257 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_34                   (258U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_34, Dest signal name: cpu0_intr_258 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_35                   (259U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_35, Dest signal name: cpu0_intr_259 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_36                   (260U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_36, Dest signal name: cpu0_intr_260 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_37                   (261U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_37, Dest signal name: cpu0_intr_261 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_38                   (262U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_38, Dest signal name: cpu0_intr_262 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_39                   (263U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_39, Dest signal name: cpu0_intr_263 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_40                   (264U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_40, Dest signal name: cpu0_intr_264 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_41                   (265U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_41, Dest signal name: cpu0_intr_265 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_42                   (266U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_42, Dest signal name: cpu0_intr_266 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_43                   (267U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_43, Dest signal name: cpu0_intr_267 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_44                   (268U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_44, Dest signal name: cpu0_intr_268 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_45                   (269U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_45, Dest signal name: cpu0_intr_269 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_46                   (270U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_46, Dest signal name: cpu0_intr_270 */
#define CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_47                   (271U)  /* Design details: Source signal name: main2mcu_pls_introuter_main_0_outp_47, Dest signal name: cpu0_intr_271 */
#define CSL_MCU0_INTR_COREPAC_CLUSTER0_INT8                          (288U)  /* Dest signal name: cpu0_intr_288 */
#define CSL_MCU0_INTR_COREPAC_CLUSTER0_INT9                          (289U)  /* Dest signal name: cpu0_intr_289 */
#define CSL_MCU0_INTR_COREPAC_CLUSTER0_INT10                         (290U)  /* Dest signal name: cpu0_intr_290 */
#define CSL_MCU0_INTR_COREPAC_CLUSTER0_ARM0_LBIST_DONE               (291U)  /* Dest signal name: cpu0_intr_291 */
#define CSL_MCU0_INTR_COREPAC_CLUSTER0_ARM1_LBIST_DONE               (292U)  /* Dest signal name: cpu0_intr_292 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_0                                (320U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_0, Dest signal name: cpu0_intr_320 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_1                                (321U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_1, Dest signal name: cpu0_intr_321 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_2                                (322U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_2, Dest signal name: cpu0_intr_322 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_3                                (323U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_3, Dest signal name: cpu0_intr_323 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_4                                (324U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_4, Dest signal name: cpu0_intr_324 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_5                                (325U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_5, Dest signal name: cpu0_intr_325 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_6                                (326U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_6, Dest signal name: cpu0_intr_326 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_7                                (327U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_7, Dest signal name: cpu0_intr_327 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_8                                (328U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_8, Dest signal name: cpu0_intr_328 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_9                                (329U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_9, Dest signal name: cpu0_intr_329 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_10                               (330U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_10, Dest signal name: cpu0_intr_330 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_11                               (331U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_11, Dest signal name: cpu0_intr_331 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_12                               (332U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_12, Dest signal name: cpu0_intr_332 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_13                               (333U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_13, Dest signal name: cpu0_intr_333 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_14                               (334U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_14, Dest signal name: cpu0_intr_334 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_15                               (335U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_15, Dest signal name: cpu0_intr_335 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_16                               (336U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_16, Dest signal name: cpu0_intr_336 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_17                               (337U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_17, Dest signal name: cpu0_intr_337 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_18                               (338U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_18, Dest signal name: cpu0_intr_338 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_19                               (339U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_19, Dest signal name: cpu0_intr_339 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_20                               (340U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_20, Dest signal name: cpu0_intr_340 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_21                               (341U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_21, Dest signal name: cpu0_intr_341 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_22                               (342U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_22, Dest signal name: cpu0_intr_342 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_23                               (343U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_23, Dest signal name: cpu0_intr_343 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_24                               (344U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_24, Dest signal name: cpu0_intr_344 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_25                               (345U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_25, Dest signal name: cpu0_intr_345 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_26                               (346U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_26, Dest signal name: cpu0_intr_346 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_27                               (347U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_27, Dest signal name: cpu0_intr_347 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_28                               (348U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_28, Dest signal name: cpu0_intr_348 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_29                               (349U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_29, Dest signal name: cpu0_intr_349 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_30                               (350U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_30, Dest signal name: cpu0_intr_350 */
#define CSL_MCU0_INTR_DMSC0_IA_PEND_31                               (351U)  /* Design details: Source signal name: dmsc_wkup_0_ia_pend_31, Dest signal name: cpu0_intr_351 */

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_MCU0_H_ */
