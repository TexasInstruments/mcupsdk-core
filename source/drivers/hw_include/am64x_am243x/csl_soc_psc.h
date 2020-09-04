/********************************************************************
*
* SOC PSC PD and LPSC ID definitions. header file
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
#ifndef CSL_SOC_PSC_H_
#define CSL_SOC_PSC_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* Auto-generated PSC definitions File
*/

/*
* PD Indices:
*/

/*
*  MCU PD
*/

#define CSL_MCU_GP_CORE_CTL_MCU                                                                    (0U)
#define CSL_MCU_PD_M4F                                                                             (1U)
/*
*  MAIN PD
*/

#define CSL_MAIN_GP_CORE_CTL                                                                       (0U)
#define CSL_MAIN_PD_A53_CLUSTER_0                                                                  (1U)
#define CSL_MAIN_PD_A53_0                                                                          (2U)
#define CSL_MAIN_PD_A53_1                                                                          (3U)
#define CSL_MAIN_PD_PULSAR_0                                                                       (4U)
#define CSL_MAIN_PD_PULSAR_1                                                                       (5U)
#define CSL_MAIN_PD_ICSSG_0                                                                        (6U)
#define CSL_MAIN_PD_ICSSG_1                                                                        (7U)
#define CSL_MAIN_PD_SPARE2                                                                         (8U)
#define CSL_MAIN_PD_CPSW                                                                           (9U)
#define CSL_MAIN_PD_SPARE0                                                                         (10U)
#define CSL_MAIN_PD_SPARE1                                                                         (11U)
/*
* LPSC Modules:
*/

/*
*  MCU LPSC
*/

#define CSL_MCU_LPSC_MCU_ALWAYSON                                                                  (0U)
#define CSL_MCU_LPSC_SPARE2                                                                        (1U)
#define CSL_MCU_LPSC_MCU_TEST                                                                      (2U)
#define CSL_MCU_LPSC_MAIN2MCU                                                                      (3U)
#define CSL_MCU_LPSC_MCU2MAIN                                                                      (4U)
#define CSL_MCU_LPSC_MCU_SPARE0                                                                    (5U)
#define CSL_MCU_LPSC_MCU_SPARE1                                                                    (6U)
#define CSL_MCU_LPSC_M4F                                                                           (7U)
/*
*  MAIN LPSC
*/

#define CSL_MAIN_LPSC_MAIN_ALWAYSON                                                                (0U)
#define CSL_MAIN_LPSC_MAIN_TEST                                                                    (1U)
#define CSL_MAIN_LPSC_MAIN_PBIST                                                                   (2U)
#define CSL_MAIN_LPSC_DMSC                                                                         (3U)
#define CSL_MAIN_LPSC_MMC4B_0                                                                      (4U)
#define CSL_MAIN_LPSC_MMC8B_0                                                                      (5U)
#define CSL_MAIN_LPSC_USB                                                                          (6U)
#define CSL_MAIN_LPSC_ADC                                                                          (7U)
#define CSL_MAIN_LPSC_DEBUGSS                                                                      (8U)
#define CSL_MAIN_LPSC_GPMC                                                                         (9U)
#define CSL_MAIN_LPSC_EMIF_CFG_0                                                                   (10U)
#define CSL_MAIN_LPSC_EMIF_DATA_0                                                                  (11U)
#define CSL_MAIN_LPSC_MCAN_0                                                                       (12U)
#define CSL_MAIN_LPSC_MCAN_1                                                                       (13U)
#define CSL_MAIN_LPSC_SAUL                                                                         (14U)
#define CSL_MAIN_LPSC_SERDES_0                                                                     (15U)
#define CSL_MAIN_LPSC_PCIE_0                                                                       (16U)
#define CSL_MAIN_LPSC_MAIN_RESERVED_0                                                              (17U)
#define CSL_MAIN_LPSC_MAIN_RESERVED_1                                                              (18U)
#define CSL_MAIN_LPSC_MAIN_RESERVED_2                                                              (19U)
#define CSL_MAIN_LPSC_A53_CLUSTER_0                                                                (20U)
#define CSL_MAIN_LPSC_A53_CLUSTER_0_PBIST                                                          (21U)
#define CSL_MAIN_LPSC_A53_0                                                                        (22U)
#define CSL_MAIN_LPSC_A53_1                                                                        (23U)
#define CSL_MAIN_LPSC_PULSAR_0_R5_0                                                                (24U)
#define CSL_MAIN_LPSC_PULSAR_0_R5_1                                                                (25U)
#define CSL_MAIN_LPSC_PULSAR_PBIST_0                                                               (26U)
#define CSL_MAIN_LPSC_PULSAR_1_R5_0                                                                (27U)
#define CSL_MAIN_LPSC_PULSAR_1_R5_1                                                                (28U)
#define CSL_MAIN_LPSC_PULSAR_PBIST_1                                                               (29U)
#define CSL_MAIN_LPSC_ICSSG_0                                                                      (30U)
#define CSL_MAIN_LPSC_ICSSG_1                                                                      (31U)
#define CSL_MAIN_LPSC_SPARE_2                                                                      (32U)
#define CSL_MAIN_LPSC_CPSW3G                                                                       (33U)
#define CSL_MAIN_LPSC_SPARE_0                                                                      (34U)
#define CSL_MAIN_LPSC_SPARE_1                                                                      (35U)

#ifdef __cplusplus
}
#endif
#endif /* CSL_SOC_PSC_H_ */

