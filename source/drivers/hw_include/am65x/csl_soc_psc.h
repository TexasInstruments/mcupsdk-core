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
#ifndef _CSL_SOC_PSC_H_
#define _CSL_SOC_PSC_H_


#ifdef __cplusplus
extern "C"
{
#endif
/*
 * Auto-generated CSL section for MAIN_PSC:
*/
/*
 * PD Indices:
*/
#define CSL_MAIN_GP_CORE_CTL                                          (0U)
#define CSL_MAIN_PD_CC_TOP                                            (1U)
#define CSL_MAIN_PD_SERDES                                            (10U)
#define CSL_MAIN_PD_ICSSG                                             (11U)
#define CSL_MAIN_PD_GPU                                               (12U)
#define CSL_MAIN_PD_EMIF                                              (13U)
#define CSL_MAIN_PD_A53_0                                             (2U)
#define CSL_MAIN_PD_A53_1                                             (3U)
#define CSL_MAIN_PD_A53_2                                             (4U)
#define CSL_MAIN_PD_A53_3                                             (5U)
#define CSL_MAIN_PD_A53_CLUSTER_0                                     (6U)
#define CSL_MAIN_PD_A53_CLUSTER_1                                     (7U)
#define CSL_MAIN_PD_DEBUG                                             (8U)
#define CSL_MAIN_PD_PER                                               (9U)
/*
 * LPSC Indices:
*/
#define CSL_MAIN_GP_CORE_CTL_LPSC_MAIN_INFRA                          (0U)
#define CSL_MAIN_GP_CORE_CTL_LPSC_MAIN_TEST                           (1U)
#define CSL_MAIN_PD_A53_3_LPSC_A53_3                                  (10U)
#define CSL_MAIN_PD_A53_CLUSTER_0_LPSC_A53_CLUSTER_0                  (11U)
#define CSL_MAIN_PD_A53_CLUSTER_0_LPSC_A53_CLUSTER_0_PBIST            (12U)
#define CSL_MAIN_PD_A53_CLUSTER_1_LPSC_A53_CLUSTER_1                  (13U)
#define CSL_MAIN_PD_A53_CLUSTER_1_LPSC_A53_CLUSTER_1_PBIST            (14U)
#define CSL_MAIN_PD_DEBUG_LPSC_MAIN_DEBUG                             (15U)
#define CSL_MAIN_PD_PER_LPSC_DSS                                      (16U)
#define CSL_MAIN_PD_PER_LPSC_MMC                                      (17U)
#define CSL_MAIN_PD_PER_LPSC_CAL                                      (18U)
#define CSL_MAIN_PD_PER_LPSC_PCIE_0                                   (19U)
#define CSL_MAIN_GP_CORE_CTL_LPSC_MAIN_PBIST                          (2U)
#define CSL_MAIN_PD_PER_LPSC_PCIE_1                                   (20U)
#define CSL_MAIN_PD_PER_LPSC_USB_0                                    (21U)
#define CSL_MAIN_PD_PER_LPSC_USB_1                                    (22U)
#define CSL_MAIN_PD_PER_LPSC_SAUL                                     (23U)
#define CSL_MAIN_PD_PER_LPSC_PER_COMMON                               (24U)
#define CSL_MAIN_PD_PER_LPSC_NB                                       (25U)
#define CSL_MAIN_PD_SERDES_LPSC_SERDES_0                              (27U)
#define CSL_MAIN_PD_SERDES_LPSC_SERDES_1                              (28U)
#define CSL_MAIN_PD_ICSSG_LPSC_ICSSG_0                                (29U)
#define CSL_MAIN_PD_ICSSG_LPSC_ICSSG_1                                (30U)
#define CSL_MAIN_PD_ICSSG_LPSC_ICSSG_2                                (31U)
#define CSL_MAIN_PD_GPU_LPSC_GPU                                      (32U)
#define CSL_MAIN_PD_GPU_LPSC_GPU_PBIST                                (33U)
#define CSL_MAIN_PD_EMIF_LPSC_EMIF_CFG                                (35U)
#define CSL_MAIN_PD_CC_TOP_LPSC_CC_TOP                                (5U)
#define CSL_MAIN_PD_CC_TOP_LPSC_CC_TOP_PBIST                          (6U)
#define CSL_MAIN_PD_A53_0_LPSC_A53_0                                  (7U)
#define CSL_MAIN_PD_A53_1_LPSC_A53_1                                  (8U)
#define CSL_MAIN_PD_A53_2_LPSC_A53_2                                  (9U)
/*
 * Auto-generated CSL section for WKUP_PSC:
*/
/*
 * PD Indices:
*/
#define CSL_WKUP_PD_WKUP                                              (0U)
#define CSL_WKUP_PD_MCU                                               (1U)
#define CSL_WKUP_PD_MCU_PULSAR                                        (2U)
/*
 * LPSC Indices:
*/
#define CSL_WKUP_PD_WKUP_LPSC_WKUP_COMMON                             (0U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_COMMON                               (10U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_TEST                                 (11U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_MCAN_0                               (12U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_MCAN_1                               (13U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_OSPI_0                               (14U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_OSPI_1                               (15U)
#define CSL_WKUP_PD_MCU_LPSC_MCU_HYPERBUS                             (16U)
#define CSL_WKUP_PD_MCU_PULSAR_LPSC_MCU_R5_0                          (19U)
#define CSL_WKUP_PD_MCU_PULSAR_LPSC_MCU_R5_1                          (20U)
#define CSL_WKUP_PD_WKUP_LPSC_WKUP_GPIO                               (5U)
#ifdef __cplusplus
}
#endif


#endif /*_CSL_SOC_PSC_H_ */
