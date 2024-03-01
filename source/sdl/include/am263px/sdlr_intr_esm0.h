/********************************************************************
*
* ESM0 INTERRUPT MAP. header file
*
* Copyright (C) 2022-2024 Texas Instruments Incorporated.
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
#ifndef SDLR_ESM0_INTERRUPT_MAP_H_
#define SDLR_ESM0_INTERRUPT_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: ESM0
*/

/* Level events */

#define SDL_ESM0_EFC_ERROR                                           0U
#define SDL_ESM0_EFUSE_EFS_AUTOLOAD_ERROR                            1U
#define SDL_ESM0_MCAN0_MCAN0_ECC_CORR_LVL_INT                        2U
#define SDL_ESM0_MCAN0_MCAN0_ECC_UNCORR_LVL_INT                      3U
#define SDL_ESM0_MCAN1_MCAN1_ECC_CORR_LVL_INT                        4U
#define SDL_ESM0_MCAN1_MCAN1_ECC_UNCORR_LVL_INT                      5U
#define SDL_ESM0_MCAN2_MCAN2_ECC_CORR_LVL_INT                        6U
#define SDL_ESM0_MCAN2_MCAN2_ECC_UNCORR_LVL_INT                      7U
#define SDL_ESM0_MCAN3_MCAN3_ECC_CORR_LVL_INT                        8U
#define SDL_ESM0_MCAN3_MCAN3_ECC_UNCORR_LVL_INT                      9U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_LIVELOCK_0                     10U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_LIVELOCK_1                     11U
#define SDL_ESM0_R5FSS1_CORE0_R5FSS1_LIVELOCK_0                     12U
#define SDL_ESM0_R5FSS1_CORE1_R5FSS1_LIVELOCK_1                     13U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_CORE0_TCMADDR_ERR              14U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_CORE1_TCMADDR_ERR              15U
#define SDL_ESM0_R5FSS1_CORE0_R5FSS1_CORE0_TCMADDR_ERR              16U
#define SDL_ESM0_R5FSS1_CORE1_R5FSS1_CORE1_TCMADDR_ERR              17U
#define SDL_ESM0_RESERVED18                                         18U
#define SDL_ESM0_ECC_AGGREGATOR_SOC_ECCAGG_CORR_LEVEL               19U
#define SDL_ESM0_ECC_AGGREGATOR_SOC_ECCAGG_UNCORR_LEVEL             20U
#define SDL_ESM0_DCC0_DCC0_ERR                                      21U
#define SDL_ESM0_DCC1_DCC1_ERR                                      22U
#define SDL_ESM0_DCC2_DCC2_ERR                                      23U
#define SDL_ESM0_DCC3_DCC3_ERR                                      24U
#define SDL_ESM0_CORE_PLL_PLL_CORE_LOCKLOSS                         25U
#define SDL_ESM0_PERI_PLL_PLL_PER_LOCKLOSS                          26U
#define SDL_ESM0_RCOSC_RCREF_CLK_LOSS_DETECT                        27U
#define SDL_ESM0_HSM_ESM_HIGH_INTR                                  28U
#define SDL_ESM0_HSM_ESM_LOW_INTR                                   29U
#define SDL_ESM0_XTAL_CRYSTAL_CLOCKLOSS                             30U
#define SDL_ESM0_AGGREGATED_VBUSP_ERROR_AGGREGATED_VBUSP_ERROR_H    31U
#define SDL_ESM0_RESERVED32                                         32U
#define SDL_ESM0_AGGREGATED_VBUSM_RRROR_AGGREGATED_VBUSM_ERROR_H    33U
#define SDL_ESM0_AGGREGATED_VBUSM_RRROR_AGGREGATED_VBUSM_ERROR_L    34U
#define SDL_ESM0_RESERVED35                                         35U
#define SDL_ESM0_RESERVED36                                         36U
#define SDL_ESM0_RESERVED37                                         37U
#define SDL_ESM0_RESERVED38                                         38U
#define SDL_ESM0_OSPI_ECC_UNCORR                                    39U
#define SDL_ESM0_OSPI_ECC_CORR                                      40U
#define SDL_ESM0_VOLTAGE_MONITOR_ERR_H                              41U
#define SDL_ESM0_VMON_VOLTAGE_MONITOR_ERR_L                         42U
#define SDL_ESM0_RESERVED43                                         43U
#define SDL_ESM0_THERMAL_MONITOR_CRITICAL                           44U
#define SDL_ESM0_CPSW3G_CPSW_ECC_SEC_PEND_INTR                      45U
#define SDL_ESM0_CPSW3G_CPSW_ECC_DED_PEND_INTR                      46U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_CORE0_ECC_CORRECTED_LEVEL_0    47U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_CORE0_ECC_UNCORRECTED_LEVEL_0  48U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_CORE1_ECC_CORRECTED_LEVEL_0    49U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_CORE1_ECC_UNCORRECTED_LEVEL_0  50U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_ECC_DE_TO_ESM_0_0              51U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_ECC_DE_TO_ESM_1_0              52U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS0_ECC_SE_TO_ESM_0_0              53U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS0_ECC_SE_TO_ESM_1_0              54U
#define SDL_ESM0_R5FSS1_CORE0_R5FSS1_CORE0_ECC_CORRECTED_LEVEL_0    55U
#define SDL_ESM0_R5FSS1_CORE0_R5FSS1_CORE0_ECC_UNCORRECTED_LEVEL_0  56U
#define SDL_ESM0_R5FSS1_CORE1_R5FSS1_CORE1_ECC_CORRECTED_LEVEL_0    57U
#define SDL_ESM0_R5FSS1_CORE1_R5FSS1_CORE1_ECC_UNCORRECTED_LEVEL_0  58U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS1_ECC_DE_TO_ESM_0_0              59U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS1_ECC_DE_TO_ESM_1_0              60U
#define SDL_ESM0_R5FSS0_CORE0_R5FSS1_ECC_SE_TO_ESM_0_0              61U
#define SDL_ESM0_R5FSS0_CORE1_R5FSS1_ECC_SE_TO_ESM_1_0              62U
#define SDL_ESM0_EDMA0_TPCC_ERRINTAGG                               63U

/* Pulse Events */
#define SDL_ESM0_RTI0_WWD_NMI                                       64U
#define SDL_ESM0_WWDT1_RTI1_WWD_NMI                                 65U
#define SDL_ESM0_WWDT2_RTI2_WWD_NMI                                 66U
#define SDL_ESM0_WWDT3_RTI3_WWD_NMI                                 67U
#define SDL_ESM0_EDMA0_TPCC_ERRINT                                  68U
#define SDL_ESM0_R5FSS0_R5FSS0_BUS_MONITOR_ERR_PULSE_0              69U
#define SDL_ESM0_R5FSS0_R5FSS0_COMPARE_ERR_PULSE_0                  70U
#define SDL_ESM0_R5FSS0_R5FSS0_VIM_COMPARE_ERR_PULSE_0              71U
#define SDL_ESM0_R5FSS0_R5FSS0_CPU_MISCOMPARE_PULSE_0               72U
#define SDL_ESM0_R5FSS1_R5FSS1_BUS_MONITOR_ERR_PULSE_0              73U
#define SDL_ESM0_R5FSS1_R5FSS1_COMPARE_ERR_PULSE_0                  74U
#define SDL_ESM0_R5FSS1_R5FSS1_VIM_COMPARE_ERR_PULSE_0              75U
#define SDL_ESM0_R5FSS1_R5FSS1_CPU_MISCOMPARE_PULSE_0               76U
#define SDL_ESM0_PRU_ICSSM0_PR1_ECC_DED_ERR_REQ                     77U
#define SDL_ESM0_PRU_ICSSM0_PR1_ECC_SEC_ERR_REQ                     78U
#define SDL_ESM0_SRAM_BANK_0_SRAM0_ECC_UNCORR_PULSE                 79U
#define SDL_ESM0_SRAM_BANK_1_SRAM1_ECC_UNCORR_PULSE                 80U
#define SDL_ESM0_SRAM_BANK_2_SRAM2_ECC_UNCORR_PULSE                 81U
#define SDL_ESM0_SRAM_BANK_3_SRAM3_ECC_UNCORR_PULSE                 82U
#define SDL_ESM0_CCM_0_SELFTEST_ERR                                 83U
#define SDL_ESM0_CCM_0_LOCKSTEP_COMPARE_ERR                         84U
#define SDL_ESM0_CCM_1_SELFTEST_ERR                                 85U
#define SDL_ESM0_CCM_1_LOCKSTEP_COMPARE_ERR                         86U
#define SDL_ESM0_R5FSS0_TMU_COMPARE_ERR                             87U
#define SDL_ESM0_R5FSS0_CPU0_TMU_PARITY_ERR                         88U
#define SDL_ESM0_R5FSS0_CPU2_TMU_PARITY_ERR                         89U
#define SDL_ESM0_R5FSS1_TMU_COMPARE_ERR                             90U
#define SDL_ESM0_R5FSS1_CPU0_TMU_PARITY_ERR                         91U
#define SDL_ESM0_R5FSS1_CPU2_TMU_PARITY_ERR                         92U
#define SDL_ESM0_R5FSS0_RL2_COMPARE_ERR                             93U
#define SDL_ESM0_R5FSS1_RL2_COMPARE_ERR                             94U
#define SDL_ESM0_ADC_SAFETY_CHECKEVENT0                             95U
#define SDL_ESM0_ADC_SAFETY_CHECKEVENT1                             96U
#define SDL_ESM0_ADC_SAFETY_CHECKEVENT2                             97U
#define SDL_ESM0_ADC_SAFETY_CHECKEVENT3                             98U
#define SDL_ESM0_OTFA_ECC_UNCORR                                    99U
#define SDL_ESM0_OTFA_ECC_CORR                                      100U
#define SDL_ESM0_SRAM_BANK_4_SRAM4_ECC_UNCORR_PULSE                 101U
#define SDL_ESM0_SRAM_BANK_5_SRAM5_ECC_UNCORR_PULSE                 102U
#define SDL_ESM0_MCAN4_MCAN4_ECC_CORR_PLS_INT                       103U
#define SDL_ESM0_MCAN4_MCAN4_ECC_UNCORR_LVL_INT                     104U
#define SDL_ESM0_MCAN5_MCAN5_ECC_CORR_LVL_INT                       105U
#define SDL_ESM0_MCAN5_MCAN5_ECC_UNCORR_LVL_INT                     106U
#define SDL_ESM0_MCAN6_MCAN6_ECC_CORR_LVL_INT                       107U
#define SDL_ESM0_MCAN6_MCAN6_ECC_UNCORR_LVL_INT                     108U
#define SDL_ESM0_MCAN7_MCAN7_ECC_CORR_LVL_INT                       109U
#define SDL_ESM0_MCAN7_MCAN7_ECC_UNCORR_LVL_INT                     110U

#ifdef __cplusplus
}
#endif
#endif /* SDLR_ESM0_INTERRUPT_MAP_H_ */