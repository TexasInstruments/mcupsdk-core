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

#define SDL_ESM_INTR_LEVEL_EFC_ERROR                                         0U
#define SDL_ESM_INTR_LEVEL_EFS_AUTOLOAD_ERROR                                1U
#define SDL_ESM_INTR_LEVEL_MCANSS0_ECC_CORR_LVL_INT                          2U
#define SDL_ESM_INTR_LEVEL_MCANSS0_ECC_UNCORR_LVL_INT                        3U
#define SDL_ESM_INTR_LEVEL_MCANSS1_ECC_CORR_LVL_INT                          4U
#define SDL_ESM_INTR_LEVEL_MCANSS1_ECC_UNCORR_LVL_INT                        5U
#define SDL_ESM_INTR_LEVEL_SOC_ECCAGG_CORR_LEVEL                             6U
#define SDL_ESM_INTR_LEVEL_SOC_ECCAGG_UNCORR_LEVEL                           7U
#define SDL_ESM_INTR_LEVEL_DCC0_ERR                                          8U
#define SDL_ESM_INTR_LEVEL_DCC1_ERR                                          9U
#define SDL_ESM_INTR_LEVEL_DCC2_ERR                                          10U
#define SDL_ESM_INTR_LEVEL_DCC3_ERR                                          11U
#define SDL_ESM_INTR_LEVEL_PLL_CORE_LOCKLOSS                                 12U
#define SDL_ESM_INTR_LEVEL_PLL_ETH_LOCKLOSS                                  13U
#define SDL_ESM_INTR_LEVEL_PLL_PER_LOCKLOSS                                  14U
#define SDL_ESM_INTR_LEVEL_RCREF_CLK_LOSS_DETECT                             15U
#define SDL_ESM_INTR_LEVEL_HSM_ESM_HIGH_INTR                                 16U
#define SDL_ESM_INTR_LEVEL_HSM_ESM_LOW_INTR                                  17U
#define SDL_ESM_INTR_LEVEL_CRYSTAL_CLOCKLOSS                                 18U
#define SDL_ESM_INTR_LEVEL_AGGREGATED_VBUSP_ERROR_H                          19U
#define SDL_ESM_INTR_LEVEL_AGGREGATED_VBUSM_ERROR_H                          20U
#define SDL_ESM_INTR_LEVEL_AGGREGATED_VBUSM_ERROR_L                          21U
#define SDL_ESM_INTR_LEVEL_FOTA_STAT_ERR_INTR                                22U
#define SDL_ESM_INTR_LEVEL_FSS_VBUSM_TIMEOUT                                 23U
#define SDL_ESM_INTR_LEVEL_OTFA_ERROR                                        24U
#define SDL_ESM_INTR_LEVEL_OSPI_ECC_CORR_LVL_INT                             25U
#define SDL_ESM_INTR_LEVEL_OSPI_ECC_UNCORR_LVL_INT                           26U
#define SDL_ESM_INTR_LEVEL_FSAS_ECC_INTR                                     27U
#define SDL_ESM_INTR_LEVEL_VOLTAGE_MONITOR_ERR_H                             28U
#define SDL_ESM_INTR_LEVEL_VOLTAGE_MONITOR_ERR_L                             29U
#define SDL_ESM_INTR_LEVEL_THERMAL_MONITOR_CRITICAL                          30U
#define SDL_ESM_INTR_LEVEL_CPSW_ECC_SEC_PEND_INTR                            31U
#define SDL_ESM_INTR_LEVEL_CPSW_ECC_DED_PEND_INTR                            32U
#define SDL_ESM_INTR_LEVEL_R5SS0_LIVELOCK_0                                  33U
#define SDL_ESM_INTR_LEVEL_R5SS0_LIVELOCK_1                                  34U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU0_TCM_ADDR_ERR                           35U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU1_TCM_ADDR_ERR                           36U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU0_ECC_CORRECTED_LEVEL                    37U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU0_ECC_UNCORRECTED_LEVEL                  38U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU1_ECC_CORRECTED_LEVEL                    39U
#define SDL_ESM_INTR_LEVEL_R5SS0_CPU1_ECC_UNCORRECTED_LEVEL                  40U
#define SDL_ESM_INTR_LEVEL_R5SS0_ECC_DE_TO_ESM_0                             41U
#define SDL_ESM_INTR_LEVEL_R5SS0_ECC_DE_TO_ESM_1                             42U
#define SDL_ESM_INTR_LEVEL_R5SS0_ECC_SE_TO_ESM_0                             43U
#define SDL_ESM_INTR_LEVEL_R5SS0_ECC_SE_TO_ESM_1                             44U
#define SDL_ESM_INTR_LEVEL_TPCC0_ERR_INTAGG                                  45U
#define SDL_ESM_INTR_LEVEL_OSPI1_ECC_CORR_LVL_INT                            46U
#define SDL_ESM_INTR_LEVEL_OSPI1_ECC_UNCORR_LVL_INT                          47U
#define RESERVED_48                                                          48U
#define RESERVED_49                                                          49U
#define RESERVED_50                                                          50U
#define RESERVED_51                                                          51U
#define RESERVED_52                                                          52U
#define RESERVED_53                                                          53U
#define RESERVED_54                                                          54U
#define RESERVED_55                                                          55U
#define RESERVED_56                                                          56U
#define RESERVED_57                                                          57U
#define RESERVED_58                                                          58U
#define RESERVED_59                                                          59U
#define RESERVED_60                                                          60U
#define RESERVED_61                                                          61U
#define RESERVED_62                                                          62U
#define RESERVED_63                                                          63U


/* Pulse Events */
#define SDL_ESM_INTR_PULSE_RTI0_WWD_NMI                                      64U
#define SDL_ESM_INTR_PULSE_RTI1_WWD_NMI                                      65U
#define SDL_ESM_INTR_PULSE_TPCC_ERRINT                                       66U
#define SDL_ESM_INTR_PULSE_R5SS0_BUS_MONITOR_ERR_PULSE                       67U
#define SDL_ESM_INTR_PULSE_R5SS0_COMPARE_ERR_PULSE                           68U
#define SDL_ESM_INTR_PULSE_R5SS0_VIM_COMPARE_ERR_PULSE                       69U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU_MISCOMPARE_PULSE                        70U
#define SDL_ESM_INTR_PULSE_R5SS0_TMU_COMP_ERR                                71U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU0_TMU_PARITY_ERR                         72U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU1_TMU_PARITY_ERR                         73U
#define SDL_ESM_INTR_PULSE_R5SS0_RL2_COMP_ERR                                74U
#define SDL_ESM_INTR_PULSE_PR0_ECC_DED_ERR_REQ                               75U
#define SDL_ESM_INTR_PULSE_PR0_ECC_SEC_ERR_REQ                               76U
#define SDL_ESM_INTR_PULSE_PR1_ECC_DED_ERR_REQ                               77U
#define SDL_ESM_INTR_PULSE_PR1_ECC_SEC_ERR_REQ                               78U
#define SDL_ESM_INTR_PULSE_SRAM0_ECC_UNCORR_LEVEL                            79U
#define SDL_ESM_INTR_PULSE_SRAM1_ECC_UNCORR_LEVEL                            80U
#define SDL_ESM_INTR_PULSE_SRAM2_ECC_UNCORR_LEVEL                            81U
#define SDL_ESM_INTR_PULSE_CCM_0_SELFTEST_ERR                                82U
#define SDL_ESM_INTR_PULSE_CCM_0_LOCKSTEP_COMPARE_ERR                        83U
#define SDL_ESM_INTR_PULSE_ADC_SAFETY_CHECKEVENT0                            84U
#define SDL_ESM_INTR_PULSE_ADC_SAFETY_CHECKEVENT1                            85U
#define SDL_ESM_INTR_PULSE_ADC_SAFETY_CHECKEVENT2                            86U
#define SDL_ESM_INTR_PULSE_ADC_SAFETY_CHECKEVENT3                            87U
#define SDL_ESM_INTR_PULSE_FOTA_ECC_UNCORR                                   88U
#define SDL_ESM_INTR_PULSE_FOTA_ECC_CORR                                     89U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU0_RL2_ECC_UNCORR                         90U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU0_RL2_ECC_CORR                           91U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU1_RL2_ECC_UNCORR                         92U
#define SDL_ESM_INTR_PULSE_R5SS0_CPU1_RL2_ECC_CORR                           93U
#define RESERVED_94                                                          94U
#define RESERVED_95                                                          95U

#ifdef __cplusplus
}
#endif
#endif /* SDLR_ESM0_INTERRUPT_MAP_H_ */