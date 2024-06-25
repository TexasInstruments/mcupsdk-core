/********************************************************************
*
* SOC ECC AGGREGATOR PROPERTIES. header file
*
* Copyright (C) 2020-2023 Texas Instruments Incorporated.
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
#ifndef SDLR_SOC_ECC_AGGR_H_
#define SDLR_SOC_ECC_AGGR_H_

#include <sdl/sdlr.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*  Common defines */
#define SDL_ECC_AGGR_INJECT_TYPE_WITH_ERROR_CAPTURE                                                	(0U)
#define SDL_ECC_AGGR_INJECT_TYPE_INJECT_ONLY                                                       	(1U)


#define SDL_ECC_AGGR_ECC_TYPE_ECC_WRAPPER                                                          (0U)
#define SDL_ECC_AGGR_ECC_TYPE_EDC_INTERCONNECT                                                     (1U)


#define SDL_ECC_AGGR_CHECKER_TYPE_INVALID                                                          (0U)
#define SDL_ECC_AGGR_CHECKER_TYPE_EDC                                                              (1U)
#define SDL_ECC_AGGR_CHECKER_TYPE_PARITY                                                           (2U)
#define SDL_ECC_AGGR_CHECKER_TYPE_REDUNDANT                                                        (3U)


/* Properties of ECC Aggregator instance : R5FSS0_CORE0_ECC_AGGR */
#define SDL_R5FSS0_CORE0_ECC_AGGR_NUM_RAMS                                                         (28U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID                                            (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID                                            (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID                                            (2U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID                                            (3U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_ID                                          (4U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_ID                                          (5U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_ID                                          (6U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_ID                                          (7U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID                                            (8U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID                                            (9U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID                                            (10U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID                                            (11U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID                                           (12U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ROW_WIDTH                                        (28U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_SIZE                                         (448U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID                                           (13U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID                                           (14U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID                                           (15U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID                                           (16U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID                                           (17U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID                                           (18U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID                                           (19U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID                                           (20U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID                                     (21U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ECC_TYPE                                   (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_INJECT_TYPE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ACCESSIBLE                                 (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ROW_WIDTH                                  (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_SIZE                                   (16384U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID                                     (22U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ECC_TYPE                                   (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_INJECT_TYPE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ACCESSIBLE                                 (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ROW_WIDTH                                  (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_SIZE                                   (16384U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID                                    (23U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)


/* Properties of ECC Aggregator instance : R5FSS0_CORE1_ECC_AGGR */
#define SDL_R5FSS0_CORE1_ECC_AGGR_NUM_RAMS                                                         (28U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID                                            (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID                                            (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID                                            (2U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID                                            (3U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_ID                                          (4U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_ID                                          (5U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_ID                                          (6U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_ID                                          (7U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ECC_TYPE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_INJECT_TYPE                                     (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ACCESSIBLE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ROW_WIDTH                                       (64U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_SIZE                                        (4096U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID                                            (8U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID                                            (9U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID                                            (10U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID                                            (11U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID                                           (12U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ROW_WIDTH                                        (12U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_SIZE                                         (192U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID                                           (13U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID                                           (14U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID                                           (15U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID                                           (16U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID                                           (17U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID                                           (18U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID                                           (19U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID                                           (20U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ECC_TYPE                                         (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_INJECT_TYPE                                      (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ACCESSIBLE                                       (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ROW_WIDTH                                        (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_SIZE                                         (2048U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID                                     (21U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ECC_TYPE                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_INJECT_TYPE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ACCESSIBLE                                 (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ROW_WIDTH                                  (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_SIZE                                   (16384U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID                                     (22U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ECC_TYPE                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_INJECT_TYPE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ACCESSIBLE                                 (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ROW_WIDTH                                  (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_SIZE                                   (16384U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID                                    (23U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_SIZE                                  (8192U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)


/* Properties of ECC Aggregator instance : MSS_ECC_AGG_MSS */
#define SDL_MSS_ECC_AGG_MSS_NUM_RAMS                                                               (8U)

#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_RAM_ID                                                  (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_ECC_TYPE                                                (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_INJECT_TYPE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_ACCESSIBLE                                              (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_ROW_WIDTH                                               (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_RAM_SIZE                                                (524288U)


#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_RAM_ID                                                  (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_ECC_TYPE                                                (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_INJECT_TYPE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_ACCESSIBLE                                              (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_ROW_WIDTH                                               (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_RAM_SIZE                                                (458752U)


#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_ID                                                    (2U)
#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_ECC_TYPE                                                  (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_INJECT_TYPE                                               (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_ACCESSIBLE                                                (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_ROW_WIDTH                                                 (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_SIZE                                                  (16384U)


#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_RAM_ID                                                  (3U)
#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_ECC_TYPE                                                (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_INJECT_TYPE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_ACCESSIBLE                                              (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_ROW_WIDTH                                               (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_RAM_SIZE                                                (4096U)


#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_ID                                          (4U)
#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_ECC_TYPE                                        (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_INJECT_TYPE                                     (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_ACCESSIBLE                                      (1U)
#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_ROW_WIDTH                                       (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_SIZE                                        (2048U)


#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID                                                 (5U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_ECC_TYPE                                               (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_INJECT_TYPE                                            (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_ACCESSIBLE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_ROW_WIDTH                                              (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_SIZE                                               (512U)


#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_ID                                                 (6U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_ECC_TYPE                                               (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_INJECT_TYPE                                            (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_ACCESSIBLE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_ROW_WIDTH                                              (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_SIZE                                               (512U)


#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_ID                                                 (7U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_ECC_TYPE                                               (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_INJECT_TYPE                                            (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_ACCESSIBLE                                             (0U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_ROW_WIDTH                                              (64U)
#define SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_SIZE                                               (512U)


/* Properties of ECC Aggregator instance : DSS_ECC_AGG */
#define SDL_DSS_ECC_AGG_NUM_RAMS                                                                   (22U)


#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID                                                      (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_ECC_TYPE                                                    (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_INJECT_TYPE                                                 (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_ACCESSIBLE                                                  (1U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_ROW_WIDTH                                                   (64U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_SIZE                                                    (786432U)


#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID                                                      (1U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_ECC_TYPE                                                    (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_INJECT_TYPE                                                 (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_ACCESSIBLE                                                  (1U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_ROW_WIDTH                                                   (64U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_SIZE                                                    (786432U)


#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID                                                      (2U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_ECC_TYPE                                                    (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_INJECT_TYPE                                                 (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_ACCESSIBLE                                                  (1U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_ROW_WIDTH                                                   (64U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_SIZE                                                    (524288U)


#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID                                                      (3U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_ECC_TYPE                                                    (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_INJECT_TYPE                                                 (0U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_ACCESSIBLE                                                  (1U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_ROW_WIDTH                                                   (64U)
#define SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_SIZE                                                    (524288U)


#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID                                                     (4U)
#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_ECC_TYPE                                                   (0U)
#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_INJECT_TYPE                                                (0U)
#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_ACCESSIBLE                                                 (1U)
#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_ROW_WIDTH                                                  (64U)
#define SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_SIZE                                                   (8192U)


#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_RAM_ID                                                  (5U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_ECC_TYPE                                                (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_INJECT_TYPE                                             (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_ACCESSIBLE                                              (1U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_ROW_WIDTH                                               (32U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_RAM_SIZE                                                (32768)


#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_RAM_ID                                                  (6U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_ECC_TYPE                                                (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_INJECT_TYPE                                             (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_ACCESSIBLE                                              (1U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_ROW_WIDTH                                               (32U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_RAM_SIZE                                                (16384U)


#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_RAM_ID                                                  (7U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_ECC_TYPE                                                (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_INJECT_TYPE                                             (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_ACCESSIBLE                                              (1U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_ROW_WIDTH                                               (32U)
#define SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_RAM_SIZE                                                (16384U)


#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID                                                 (8U)
#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_ECC_TYPE                                               (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_INJECT_TYPE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_ACCESSIBLE                                             (1U)
#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_ROW_WIDTH                                              (64U)
#define SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_SIZE                                               (4096U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID                                                (9U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID                                                (10U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID                                                (11U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID                                                (12U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID                                                (13U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID                                                (14U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID                                                (15U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID                                                (16U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID                                                (17U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID                                                (18U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_RAM_ID                                                (19U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_RAM_ID                                                (20U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_ECC_TYPE                                              (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_INJECT_TYPE                                           (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_ACCESSIBLE                                            (0U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_ROW_WIDTH                                             (64U)
#define SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_RAM_SIZE                                              (512U)


#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID                                               (21U)
#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_ECC_TYPE                                             (0U)
#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_INJECT_TYPE                                          (0U)
#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_ACCESSIBLE                                           (1U)
#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_ROW_WIDTH                                            (64U)
#define SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_SIZE                                             (4096U)


/* Properties of ECC Aggregator instance : MSS_MCANA_ECC */
#define SDL_MSS_MCANA_ECC_NUM_RAMS                                                                 (1U)


#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID                                                     (0U)
#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_ECC_TYPE                                                   (0U)
#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_INJECT_TYPE                                                (0U)
#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_ACCESSIBLE                                                 (0U)
#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_ROW_WIDTH                                                  (32U)
#define SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_SIZE                                                   (17408U)


/* Properties of ECC Aggregator instance : MSS_MCANB_ECC */
#define SDL_MSS_MCANB_ECC_NUM_RAMS                                                                 (1U)


#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_RAM_ID                                                     (0U)
#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_ECC_TYPE                                                   (0U)
#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_INJECT_TYPE                                                (0U)
#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_ACCESSIBLE                                                 (0U)
#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_ROW_WIDTH                                                  (32U)
#define SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_RAM_SIZE                                                   (17408U)

/* Properties of ECC Aggregator instance : CPSW3GCSS_ECC_AGGR */
#define SDL_CPSW3GCSS_ECC_AGGR_NUM_RAMS                                                            (8U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_RAM_ID                                        (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_ECC_TYPE                                      (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_INJECT_TYPE                                   (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_ACCESSIBLE                                    (1U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_ROW_WIDTH                                     (568U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_RAM_SIZE                                      (4544U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_RAM_ID                              (1U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_RAM_ID                              (2U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_RAM_ID                              (3U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_RAM_ID                              (4U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_RAM_ID                              (5U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_RAM_ID                              (6U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_ECC_TYPE                            (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_INJECT_TYPE                         (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_ACCESSIBLE                          (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_ROW_WIDTH                           (256U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_RAM_SIZE                            (32U)


#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_RAM_ID                                        (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_ECC_TYPE                                      (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_INJECT_TYPE                                   (0U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_ACCESSIBLE                                    (1U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_ROW_WIDTH                                     (22U)
#define SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_RAM_SIZE                                      (704U)

/* Summary of ECC aggregators */
#define SDL_ECC_AGGR_NUM_ECC_AGGREGATORS                                                           (7U)

#ifdef __cplusplus
}
#endif
#endif /* SDLR_SOC_ECC_AGGR_H_ */
