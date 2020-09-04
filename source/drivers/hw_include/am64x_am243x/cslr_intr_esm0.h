/********************************************************************
*
* ESM0 INTERRUPT MAP. header file
*
* Copyright (C) 2015-2019 Texas Instruments Incorporated.
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
#ifndef CSLR_ESM0_INTERRUPT_MAP_H_
#define CSLR_ESM0_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: ESM0
*/

#define CSLR_ESM0_ESM_LVL_EVENT_ADC0_ECC_CORRECTED_ERR_LEVEL_0                                     (0U)
#define CSLR_ESM0_ESM_LVL_EVENT_ECC_AGGR0_CORR_LEVEL_0                                             (1U)
#define CSLR_ESM0_ESM_LVL_EVENT_ECC_AGGR1_CORR_LEVEL_0                                             (2U)
#define CSLR_ESM0_ESM_LVL_EVENT_CPSW0_ECC_SEC_PEND_0                                               (3U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_RAT_0_EXP_INTR_0                                             (4U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_RTI_0_INTR_WWD_0                                             (5U)
#define CSLR_ESM0_ESM_LVL_EVENT_DDR16SS0_DDRSS_DRAM_ECC_CORR_ERR_LVL_0                             (6U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_ECC_AGGR_0_ECC_CORRECTED_ERR_LEVEL_0                         (8U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMASS0_ECC_AGGR_0_ECC_CORRECTED_ERR_LEVEL_0                        (9U)
#define CSLR_ESM0_ESM_LVL_EVENT_FSS0_OSPI_0_OSPI_ECC_CORR_LVL_INTR_0                               (11U)
#define CSLR_ESM0_ESM_LVL_EVENT_GICSS0_ECC_AGGR_CORR_LEVEL_0                                       (12U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG0_PR1_ECC_SEC_ERR_PEND_0                                  (13U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG1_PR1_ECC_SEC_ERR_PEND_0                                  (14U)
#define CSLR_ESM0_ESM_LVL_EVENT_PDMA0_ECC_SEC_PEND_0                                               (15U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCAN0_MCANSS_ECC_CORR_LVL_INT_0                                    (16U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCAN1_MCANSS_ECC_CORR_LVL_INT_0                                    (17U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K0_ECC_CORR_LEVEL_0                                       (18U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K1_ECC_CORR_LEVEL_0                                       (19U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K2_ECC_CORR_LEVEL_0                                       (20U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K3_ECC_CORR_LEVEL_0                                       (21U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K4_ECC_CORR_LEVEL_0                                       (22U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K5_ECC_CORR_LEVEL_0                                       (23U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR0_CORRECTED_ERR_LEVEL_0                          (24U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR1_CORRECTED_ERR_LEVEL_0                          (25U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR_COREPAC_CORRECTED_ERR_LEVEL_0                   (26U)
#define CSLR_ESM0_ESM_LVL_EVENT_PCIE0_PCIE_ECC0_CORR_LEVEL_0                                       (27U)
#define CSLR_ESM0_ESM_LVL_EVENT_PDMA1_ECC_SEC_PEND_0                                               (28U)
#define CSLR_ESM0_ESM_LVL_EVENT_PSRAMECC0_ECC_CORR_LEVEL_0                                         (29U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE0_ECC_CORRECTED_LEVEL_0                                 (30U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE1_ECC_CORRECTED_LEVEL_0                                 (31U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE0_ECC_CORRECTED_LEVEL_0                                 (32U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE1_ECC_CORRECTED_LEVEL_0                                 (33U)
#define CSLR_ESM0_ESM_LVL_EVENT_SA2_UL0_SA_UL_ECC_CORR_LEVEL_0                                     (34U)
#define CSLR_ESM0_ESM_LVL_EVENT_USB0_A_ECC_AGGR_CORRECTED_ERR_LEVEL_0                              (35U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_CFG_LVL_0                                         (37U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_HI_LVL_0                                          (38U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_LOW_LVL_0                                         (39U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_DE_TO_ESM_0_0                                   (40U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_DE_TO_ESM_1_0                                   (41U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_SE_TO_ESM_0_0                                   (42U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_SE_TO_ESM_1_0                                   (43U)
#define CSLR_ESM0_ESM_LVL_EVENT_COMPUTE_CLUSTER0_PBIST_0_DFT_PBIST_SAFETY_ERROR_0                  (44U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_COMMON0_ECC_DE_TO_ESM_0_0                                   (45U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_COMMON0_ECC_DE_TO_ESM_1_0                                   (46U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_COMMON0_ECC_SE_TO_ESM_0_0                                   (47U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_COMMON0_ECC_SE_TO_ESM_1_0                                   (48U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_DMTIMER_0_INTR_PEND_0                                        (50U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_DMTIMER_1_INTR_PEND_0                                        (51U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_DMTIMER_2_INTR_PEND_0                                        (52U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_DMTIMER_3_INTR_PEND_0                                        (53U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD0_EMMCSS_RXMEM_CORR_ERR_LVL_0                                 (54U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD0_EMMCSS_RXMEM_UNCORR_ERR_LVL_0                               (55U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD0_EMMCSS_TXMEM_CORR_ERR_LVL_0                                 (56U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD0_EMMCSS_TXMEM_UNCORR_ERR_LVL_0                               (57U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD1_EMMCSDSS_RXMEM_CORR_ERR_LVL_0                               (58U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD1_EMMCSDSS_RXMEM_UNCORR_ERR_LVL_0                             (59U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD1_EMMCSDSS_TXMEM_CORR_ERR_LVL_0                               (60U)
#define CSLR_ESM0_ESM_LVL_EVENT_MMCSD1_EMMCSDSS_TXMEM_UNCORR_ERR_LVL_0                             (61U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K6_ECC_CORR_LEVEL_0                                       (62U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K7_ECC_CORR_LEVEL_0                                       (63U)
#define CSLR_ESM0_ESM_LVL_EVENT_ADC0_ECC_UNCORRECTED_ERR_LEVEL_0                                   (64U)
#define CSLR_ESM0_ESM_LVL_EVENT_ECC_AGGR0_UNCORR_LEVEL_0                                           (65U)
#define CSLR_ESM0_ESM_LVL_EVENT_ECC_AGGR1_UNCORR_LEVEL_0                                           (66U)
#define CSLR_ESM0_ESM_LVL_EVENT_CPSW0_ECC_DED_PEND_0                                               (67U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG0_PR1_EDIO0_WD_TRIIG_0                                    (68U)
#define CSLR_ESM0_ESM_LVL_EVENT_DDR16SS0_DDRSS_DRAM_ECC_UNCORR_ERR_LVL_0                           (69U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG0_PR1_EDIO1_WD_TRIIG_0                                    (70U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMSC0_ECC_AGGR_0_ECC_UNCORRECTED_ERR_LEVEL_0                       (71U)
#define CSLR_ESM0_ESM_LVL_EVENT_DMASS0_ECC_AGGR_0_ECC_UNCORRECTED_ERR_LEVEL_0                      (72U)
#define CSLR_ESM0_ESM_LVL_EVENT_FSS0_OSPI_0_OSPI_ECC_UNCORR_LVL_INTR_0                             (74U)
#define CSLR_ESM0_ESM_LVL_EVENT_GICSS0_ECC_AGGR_UNCORR_LEVEL_0                                     (75U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG0_PR1_ECC_DED_ERR_PEND_0                                  (76U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG1_PR1_ECC_DED_ERR_PEND_0                                  (77U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCAN0_MCANSS_ECC_UNCORR_LVL_INT_0                                  (78U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCAN1_MCANSS_ECC_UNCORR_LVL_INT_0                                  (79U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K0_ECC_UNCORR_LEVEL_0                                     (80U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K1_ECC_UNCORR_LEVEL_0                                     (81U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K2_ECC_UNCORR_LEVEL_0                                     (82U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K3_ECC_UNCORR_LEVEL_0                                     (83U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K4_ECC_UNCORR_LEVEL_0                                     (84U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K5_ECC_UNCORR_LEVEL_0                                     (85U)
#define CSLR_ESM0_ESM_LVL_EVENT_PCIE0_PCIE_ECC0_UNCORR_LEVEL_0                                     (86U)
#define CSLR_ESM0_ESM_LVL_EVENT_PCIE0_PCIE_ECC1_UNCORR_LEVEL_0                                     (87U)
#define CSLR_ESM0_ESM_LVL_EVENT_PDMA0_ECC_DED_PEND_0                                               (88U)
#define CSLR_ESM0_ESM_LVL_EVENT_PDMA1_ECC_DED_PEND_0                                               (89U)
#define CSLR_ESM0_ESM_LVL_EVENT_PSRAMECC0_ECC_UNCORR_LEVEL_0                                       (90U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE0_ECC_UNCORRECTED_LEVEL_0                               (91U)
#define CSLR_ESM0_ESM_LVL_EVENT_SA2_UL0_SA_UL_ECC_UNCORR_LEVEL_0                                   (92U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR1_UNCORRECTED_ERR_LEVEL_0                        (93U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR0_UNCORRECTED_ERR_LEVEL_0                        (94U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_ECC_ECCAGGR_COREPAC_UNCORRECTED_ERR_LEVEL_0                 (95U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG1_PR1_EDIO0_WD_TRIIG_0                                    (96U)
#define CSLR_ESM0_ESM_LVL_EVENT_PRU_ICSSG1_PR1_EDIO1_WD_TRIIG_0                                    (97U)
#define CSLR_ESM0_ESM_LVL_EVENT_DFTSS0_DFT_SAFETY_123_0                                            (98U)
#define CSLR_ESM0_ESM_LVL_EVENT_DFTSS0_DFT_SAFETY_MULTI_0                                          (99U)
#define CSLR_ESM0_ESM_LVL_EVENT_DFTSS0_DFT_SAFETY_ONE_0                                            (100U)
#define CSLR_ESM0_ESM_LVL_EVENT_MAIN0_VDD_CORE_GLDTC_STAT_THRESH_HI_FLAG_IPCFG_0                   (101U)
#define CSLR_ESM0_ESM_LVL_EVENT_MAIN0_VDD_CORE_GLDTC_STAT_THRESH_LOW_FLAG_IPCFG_0                  (102U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K6_ECC_UNCORR_LEVEL_0                                     (103U)
#define CSLR_ESM0_ESM_LVL_EVENT_MSRAM_256K7_ECC_UNCORR_LEVEL_0                                     (104U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE1_ECC_UNCORRECTED_LEVEL_0                               (105U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE0_ECC_UNCORRECTED_LEVEL_0                               (106U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE1_ECC_UNCORRECTED_LEVEL_0                               (107U)
#define CSLR_ESM0_ESM_LVL_EVENT_DDR16SS0_DDRSS_V2A_OTHER_ERR_LVL_0                                 (110U)
#define CSLR_ESM0_ESM_LVL_EVENT_USB0_A_ECC_AGGR_UNCORRECTED_ERR_LEVEL_0                            (111U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC0_INTR_ERR_LEVEL_0                                              (112U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC1_INTR_ERR_LEVEL_0                                              (113U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC2_INTR_ERR_LEVEL_0                                              (114U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC3_INTR_ERR_LEVEL_0                                              (115U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC4_INTR_ERR_LEVEL_0                                              (116U)
#define CSLR_ESM0_ESM_LVL_EVENT_DCC5_INTR_ERR_LEVEL_0                                              (117U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE0_EXP_INTR_0                                            (124U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE1_EXP_INTR_0                                            (125U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE0_EXP_INTR_0                                            (126U)
#define CSLR_ESM0_ESM_LVL_EVENT_R5FSS1_CORE1_EXP_INTR_0                                            (127U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD0_LOCKLOSS_IPCFG_0                                   (128U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD1_LOCKLOSS_IPCFG_0                                   (129U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD2_LOCKLOSS_IPCFG_0                                   (130U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD8_LOCKLOSS_IPCFG_0                                   (131U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD12_LOCKLOSS_IPCFG_0                                  (132U)
#define CSLR_ESM0_ESM_LVL_EVENT_PLLFRACF_SSMOD14_LOCKLOSS_IPCFG_0                                  (133U)
#define CSLR_ESM0_ESM_LVL_EVENT_MCU_PLLFRACF_SSMOD0_LOCKLOSS_IPCFG_0                               (134U)
#define CSLR_ESM0_ESM_LVL_EVENT_GLUELOGIC_HFOSC0_CLKLOSS_GLUE_REF_CLK_LOSS_DETECT_OUT_0            (135U)
#define CSLR_ESM0_ESM_LVL_EVENT_VTM0_THERM_LVL_LT_TH0_INTR_0                                       (136U)
#define CSLR_ESM0_ESM_LVL_EVENT_VTM0_THERM_LVL_GT_TH1_INTR_0                                       (137U)
#define CSLR_ESM0_ESM_LVL_EVENT_VTM0_THERM_LVL_GT_TH2_INTR_0                                       (138U)
#define CSLR_ESM0_ESM_LVL_EVENT_VTM0_CORR_LEVEL_0                                                  (139U)
#define CSLR_ESM0_ESM_LVL_EVENT_VTM0_UNCORR_LEVEL_0                                                (140U)
#define CSLR_ESM0_ESM_LVL_EVENT_FSS0_FSAS_0_ECC_INTR_ERR_PEND_0                                    (141U)
#define CSLR_ESM0_ESM_LVL_EVENT_USB0_ASF_INT_FATAL_0                                               (142U)
#define CSLR_ESM0_ESM_LVL_EVENT_USB0_ASF_INT_NONFATAL_0                                            (143U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_EXTERRIRQ_0                                                 (144U)
#define CSLR_ESM0_ESM_LVL_EVENT_A53SS0_INTERRIRQ_0                                                 (145U)
#define CSLR_ESM0_ESM_LVL_EVENT_PBIST3_DFT_PBIST_SAFETY_ERROR_0                                    (156U)
#define CSLR_ESM0_ESM_LVL_EVENT_PBIST2_DFT_PBIST_SAFETY_ERROR_0                                    (157U)
#define CSLR_ESM0_ESM_LVL_EVENT_PBIST1_DFT_PBIST_SAFETY_ERROR_0                                    (158U)
#define CSLR_ESM0_ESM_LVL_EVENT_PBIST0_DFT_PBIST_SAFETY_ERROR_0                                    (159U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI0_INTR_WWD_0                                                   (160U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI0_INTR_WWD_0                                                   (160U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI0_INTR_WWD_0                                                   (160U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI1_INTR_WWD_0                                                   (161U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI1_INTR_WWD_0                                                   (161U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI1_INTR_WWD_0                                                   (161U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI8_INTR_WWD_0                                                   (162U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI8_INTR_WWD_0                                                   (162U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI8_INTR_WWD_0                                                   (162U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI9_INTR_WWD_0                                                   (163U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI9_INTR_WWD_0                                                   (163U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI9_INTR_WWD_0                                                   (163U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI10_INTR_WWD_0                                                  (164U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI10_INTR_WWD_0                                                  (164U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI10_INTR_WWD_0                                                  (164U)
#define CSLR_ESM0_ESM_PLS_EVENT0_RTI11_INTR_WWD_0                                                  (165U)
#define CSLR_ESM0_ESM_PLS_EVENT1_RTI11_INTR_WWD_0                                                  (165U)
#define CSLR_ESM0_ESM_PLS_EVENT2_RTI11_INTR_WWD_0                                                  (165U)
#define CSLR_ESM0_ESM_PLS_EVENT0_GICSS0_AXIM_ERR_0                                                 (166U)
#define CSLR_ESM0_ESM_PLS_EVENT1_GICSS0_AXIM_ERR_0                                                 (166U)
#define CSLR_ESM0_ESM_PLS_EVENT2_GICSS0_AXIM_ERR_0                                                 (166U)
#define CSLR_ESM0_ESM_PLS_EVENT0_GICSS0_ECC_FATAL_0                                                (167U)
#define CSLR_ESM0_ESM_PLS_EVENT1_GICSS0_ECC_FATAL_0                                                (167U)
#define CSLR_ESM0_ESM_PLS_EVENT2_GICSS0_ECC_FATAL_0                                                (167U)
#define CSLR_ESM0_ESM_PLS_EVENT0_PBIST0_DFT_PBIST_CPU_0                                            (168U)
#define CSLR_ESM0_ESM_PLS_EVENT1_PBIST0_DFT_PBIST_CPU_0                                            (168U)
#define CSLR_ESM0_ESM_PLS_EVENT2_PBIST0_DFT_PBIST_CPU_0                                            (168U)
#define CSLR_ESM0_ESM_PLS_EVENT0_PBIST1_DFT_PBIST_CPU_0                                            (170U)
#define CSLR_ESM0_ESM_PLS_EVENT1_PBIST1_DFT_PBIST_CPU_0                                            (170U)
#define CSLR_ESM0_ESM_PLS_EVENT2_PBIST1_DFT_PBIST_CPU_0                                            (170U)
#define CSLR_ESM0_ESM_PLS_EVENT0_PBIST2_DFT_PBIST_CPU_0                                            (172U)
#define CSLR_ESM0_ESM_PLS_EVENT1_PBIST2_DFT_PBIST_CPU_0                                            (172U)
#define CSLR_ESM0_ESM_PLS_EVENT2_PBIST2_DFT_PBIST_CPU_0                                            (172U)
#define CSLR_ESM0_ESM_PLS_EVENT0_PBIST3_DFT_PBIST_CPU_0                                            (174U)
#define CSLR_ESM0_ESM_PLS_EVENT1_PBIST3_DFT_PBIST_CPU_0                                            (174U)
#define CSLR_ESM0_ESM_PLS_EVENT2_PBIST3_DFT_PBIST_CPU_0                                            (174U)
#define CSLR_ESM0_ESM_PLS_EVENT0_COMPUTE_CLUSTER0_PBIST_0_DFT_PBIST_CPU_0                          (176U)
#define CSLR_ESM0_ESM_PLS_EVENT1_COMPUTE_CLUSTER0_PBIST_0_DFT_PBIST_CPU_0                          (176U)
#define CSLR_ESM0_ESM_PLS_EVENT2_COMPUTE_CLUSTER0_PBIST_0_DFT_PBIST_CPU_0                          (176U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_ESM0_INTERRUPT_MAP_H_ */

