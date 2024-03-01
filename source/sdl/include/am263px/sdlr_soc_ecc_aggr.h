/********************************************************************
*
* SOC ECC AGGREGATOR PROPERTIES. header file
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
#ifndef SDLR_SOC_ECC_AGGR_H_
#define SDLR_SOC_ECC_AGGR_H_

#include <sdl/sdlr.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*  Common defines */
#define SDL_ECC_AGGR_INJECT_TYPE_WITH_ERROR_CAPTURE                                                (0U)
#define SDL_ECC_AGGR_INJECT_TYPE_INJECT_ONLY                                                       (1U)


#define SDL_ECC_AGGR_ECC_TYPE_ECC_WRAPPER                                                          (0U)
#define SDL_ECC_AGGR_ECC_TYPE_EDC_INTERCONNECT                                                     (1U)


#define SDL_ECC_AGGR_CHECKER_TYPE_INVALID                                                          (0U)
#define SDL_ECC_AGGR_CHECKER_TYPE_EDC                                                              (1U)
#define SDL_ECC_AGGR_CHECKER_TYPE_PARITY                                                           (2U)
#define SDL_ECC_AGGR_CHECKER_TYPE_REDUNDANT                                                        (3U)


/* Properties of ECC Aggregator instance : SOC_ECC_AGGR */
#define SDL_SOC_ECC_AGGR_NUM_RAMS                                                                  (9U)


#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID                                                    (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_SIZE                                                  (524288U)


#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID                                                    (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_SIZE                                                  (524288U)


#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID                                                    (2U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_SIZE                                                  (524288U)


#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID                                                    (3U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_SIZE                                                  (524288U)


#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID                                                        (4U)
#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_ECC_TYPE                                                      (0U)
#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_INJECT_TYPE                                                   (0U)
#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_ACCESSIBLE                                                    (1U)
#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_ROW_WIDTH                                                     (64U)
#define SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_SIZE                                                      (524288U)


#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID                                                        (5U)
#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_ECC_TYPE                                                      (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_INJECT_TYPE                                                   (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_ACCESSIBLE                                                    (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_ROW_WIDTH                                                     (64U)
#define SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_SIZE                                                      (524288U)


#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_ID                                                        (6U)
#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_ECC_TYPE                                                      (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_INJECT_TYPE                                                   (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_ACCESSIBLE                                                    (0U)
#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_ROW_WIDTH                                                     (64U)
#define SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_SIZE                                                      (524288U)

#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_RAM_ID                                                    (7U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV4_ECC_RAM_SIZE                                                  (524288U)

#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_RAM_ID                                                    (8U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_ECC_TYPE                                                  (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_INJECT_TYPE                                               (0U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_ACCESSIBLE                                                (1U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_ROW_WIDTH                                                 (64U)
#define SDL_SOC_ECC_AGGR_MSS_L2_SLV5_ECC_RAM_SIZE                                                  (524288U)

/* Properties of ECC Aggregator instance : R5FSS0_CORE0_ECC_AGGR */
#define SDL_R5FSS0_CORE0_ECC_AGGR_NUM_RAMS                                                         (29U)


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
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)

#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_RAM_ID                                        (28U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ACCESSIBLE                                    (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ROW_WIDTH                                     (512U)
#define SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_RAM_SIZE                                      (136U)


/* Properties of ECC Aggregator instance : R5FSS0_CORE1_ECC_AGGR */
#define SDL_R5FSS0_CORE1_ECC_AGGR_NUM_RAMS                                                         (29U)


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
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)

#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_RAM_ID                                        (28U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ACCESSIBLE                                    (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ROW_WIDTH                                     (512U)
#define SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_RAM_SIZE                                      (136U)

/* Properties of ECC Aggregator instance : R5FSS1_CORE0_ECC_AGGR */
#define SDL_R5FSS1_CORE0_ECC_AGGR_NUM_RAMS                                                         (29U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID                                            (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID                                            (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID                                            (2U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID                                            (3U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_ID                                          (4U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_ID                                          (5U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_ID                                          (6U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_ID                                          (7U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID                                            (8U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID                                            (9U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID                                            (10U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID                                            (11U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID                                           (12U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_ROW_WIDTH                                        (28U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_SIZE                                         (448U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID                                           (13U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID                                           (14U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID                                           (15U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID                                           (16U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID                                           (17U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID                                           (18U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID                                           (19U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID                                           (20U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID                                     (21U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ECC_TYPE                                   (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_INJECT_TYPE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ACCESSIBLE                                 (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_ROW_WIDTH                                  (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_SIZE                                   (16384U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID                                     (22U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ECC_TYPE                                   (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_INJECT_TYPE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ACCESSIBLE                                 (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_ROW_WIDTH                                  (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_SIZE                                   (16384U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID                                    (23U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)

#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_RAM_ID                                        (28U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ACCESSIBLE                                    (0U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_ROW_WIDTH                                     (512U)
#define SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_RL2_RAMECC_RAM_SIZE                                      (136U)


/* Properties of ECC Aggregator instance : R5FSS1_CORE1_ECC_AGGR */
#define SDL_R5FSS1_CORE1_ECC_AGGR_NUM_RAMS                                                         (29U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID                                            (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID                                            (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID                                            (2U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID                                            (3U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_ID                                          (4U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_ID                                          (5U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_ID                                          (6U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_ID                                          (7U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ECC_TYPE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_INJECT_TYPE                                     (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ACCESSIBLE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_ROW_WIDTH                                       (64U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_SIZE                                        (4096U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID                                            (8U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID                                            (9U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID                                            (10U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID                                            (11U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ECC_TYPE                                          (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_INJECT_TYPE                                       (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ACCESSIBLE                                        (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_ROW_WIDTH                                         (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_SIZE                                          (336U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID                                           (12U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_ROW_WIDTH                                        (12U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_SIZE                                         (192U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID                                           (13U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID                                           (14U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID                                           (15U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID                                           (16U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID                                           (17U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID                                           (18U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID                                           (19U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID                                           (20U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ECC_TYPE                                         (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_INJECT_TYPE                                      (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ACCESSIBLE                                       (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_ROW_WIDTH                                        (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_SIZE                                         (2048U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID                                     (21U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ECC_TYPE                                   (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_INJECT_TYPE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ACCESSIBLE                                 (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_ROW_WIDTH                                  (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_SIZE                                   (16384U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID                                     (22U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ECC_TYPE                                   (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_INJECT_TYPE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ACCESSIBLE                                 (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_ROW_WIDTH                                  (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_SIZE                                   (16384U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID                                    (23U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID                                    (24U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID                                    (25U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID                                    (26U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ECC_TYPE                                  (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_INJECT_TYPE                               (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ACCESSIBLE                                (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_ROW_WIDTH                                 (32U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_SIZE                                  (24576U)


#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_ID                                        (27U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ACCESSIBLE                                    (1U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_ROW_WIDTH                                     (30U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_SIZE                                      (1920U)

#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_RAM_ID                                        (28U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ECC_TYPE                                      (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_INJECT_TYPE                                   (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ACCESSIBLE                                    (0U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_ROW_WIDTH                                     (512U)
#define SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_RL2_RAMECC_RAM_SIZE                                      (136U)

/* Properties of ECC Aggregator instance : HSM_ECC_AGGR */
#define SDL_HSM_ECC_AGGR_NUM_RAMS                                                                  (10U)


#define SDL_HSM_ECC_AGGR_RAMB0_RAM_ID                                                              (0U)
#define SDL_HSM_ECC_AGGR_RAMB0_ECC_TYPE                                                            (0U)
#define SDL_HSM_ECC_AGGR_RAMB0_INJECT_TYPE                                                         (0U)
#define SDL_HSM_ECC_AGGR_RAMB0_ACCESSIBLE                                                          (0U)
#define SDL_HSM_ECC_AGGR_RAMB0_ROW_WIDTH                                                           (32U)
#define SDL_HSM_ECC_AGGR_RAMB0_RAM_SIZE                                                            (65536U)


#define SDL_HSM_ECC_AGGR_RAMB1_RAM_ID                                                              (1U)
#define SDL_HSM_ECC_AGGR_RAMB1_ECC_TYPE                                                            (0U)
#define SDL_HSM_ECC_AGGR_RAMB1_INJECT_TYPE                                                         (0U)
#define SDL_HSM_ECC_AGGR_RAMB1_ACCESSIBLE                                                          (0U)
#define SDL_HSM_ECC_AGGR_RAMB1_ROW_WIDTH                                                           (32U)
#define SDL_HSM_ECC_AGGR_RAMB1_RAM_SIZE                                                            (65536U)


#define SDL_HSM_ECC_AGGR_RAMB2_RAM_ID                                                              (2U)
#define SDL_HSM_ECC_AGGR_RAMB2_ECC_TYPE                                                            (0U)
#define SDL_HSM_ECC_AGGR_RAMB2_INJECT_TYPE                                                         (0U)
#define SDL_HSM_ECC_AGGR_RAMB2_ACCESSIBLE                                                          (0U)
#define SDL_HSM_ECC_AGGR_RAMB2_ROW_WIDTH                                                           (32U)
#define SDL_HSM_ECC_AGGR_RAMB2_RAM_SIZE                                                            (65536U)


#define SDL_HSM_ECC_AGGR_RAMB3_RAM_ID                                                              (3U)
#define SDL_HSM_ECC_AGGR_RAMB3_ECC_TYPE                                                            (0U)
#define SDL_HSM_ECC_AGGR_RAMB3_INJECT_TYPE                                                         (0U)
#define SDL_HSM_ECC_AGGR_RAMB3_ACCESSIBLE                                                          (0U)
#define SDL_HSM_ECC_AGGR_RAMB3_ROW_WIDTH                                                           (32U)
#define SDL_HSM_ECC_AGGR_RAMB3_RAM_SIZE                                                            (65536U)


#define SDL_HSM_ECC_AGGR_SECUREB4_RAM_ID                                                           (4U)
#define SDL_HSM_ECC_AGGR_SECUREB4_ECC_TYPE                                                         (0U)
#define SDL_HSM_ECC_AGGR_SECUREB4_INJECT_TYPE                                                      (0U)
#define SDL_HSM_ECC_AGGR_SECUREB4_ACCESSIBLE                                                       (0U)
#define SDL_HSM_ECC_AGGR_SECUREB4_ROW_WIDTH                                                        (32U)
#define SDL_HSM_ECC_AGGR_SECUREB4_RAM_SIZE                                                         (10240U)


#define SDL_HSM_ECC_AGGR_MBOX_RAM_ID                                                               (5U)
#define SDL_HSM_ECC_AGGR_MBOX_ECC_TYPE                                                             (0U)
#define SDL_HSM_ECC_AGGR_MBOX_INJECT_TYPE                                                          (0U)
#define SDL_HSM_ECC_AGGR_MBOX_ACCESSIBLE                                                           (0U)
#define SDL_HSM_ECC_AGGR_MBOX_ROW_WIDTH                                                            (32U)
#define SDL_HSM_ECC_AGGR_MBOX_RAM_SIZE                                                             (2048U)


#define SDL_HSM_ECC_AGGR_SECURE_RAM_ID                                                             (6U)
#define SDL_HSM_ECC_AGGR_SECURE_ECC_TYPE                                                           (0U)
#define SDL_HSM_ECC_AGGR_SECURE_INJECT_TYPE                                                        (0U)
#define SDL_HSM_ECC_AGGR_SECURE_ACCESSIBLE                                                         (0U)
#define SDL_HSM_ECC_AGGR_SECURE_ROW_WIDTH                                                          (32U)
#define SDL_HSM_ECC_AGGR_SECURE_RAM_SIZE                                                           (32768U)


#define SDL_HSM_ECC_AGGR_ROM_RAM_ID                                                                (7U)
#define SDL_HSM_ECC_AGGR_ROM_ECC_TYPE                                                              (0U)
#define SDL_HSM_ECC_AGGR_ROM_INJECT_TYPE                                                           (0U)
#define SDL_HSM_ECC_AGGR_ROM_ACCESSIBLE                                                            (0U)
#define SDL_HSM_ECC_AGGR_ROM_ROW_WIDTH                                                             (32U)
#define SDL_HSM_ECC_AGGR_ROM_RAM_SIZE                                                              (32768U)


#define SDL_HSM_ECC_AGGR_TPTC_A0_RAM_ID                                                            (8U)
#define SDL_HSM_ECC_AGGR_TPTC_A0_ECC_TYPE                                                          (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A0_INJECT_TYPE                                                       (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A0_ACCESSIBLE                                                        (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A0_ROW_WIDTH                                                         (64U)
#define SDL_HSM_ECC_AGGR_TPTC_A0_RAM_SIZE                                                          (128U)


#define SDL_HSM_ECC_AGGR_TPTC_A1_RAM_ID                                                            (9U)
#define SDL_HSM_ECC_AGGR_TPTC_A1_ECC_TYPE                                                          (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A1_INJECT_TYPE                                                       (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A1_ACCESSIBLE                                                        (0U)
#define SDL_HSM_ECC_AGGR_TPTC_A1_ROW_WIDTH                                                         (64U)
#define SDL_HSM_ECC_AGGR_TPTC_A1_RAM_SIZE                                                          (128U)


/* Properties of ECC Aggregator instance : PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR */
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_NUM_RAMS                                           (5U)


#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID                       (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_ECC_TYPE                     (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_INJECT_TYPE                  (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_ACCESSIBLE                   (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_ROW_WIDTH                    (32U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_SIZE                     (8192U)


#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID                       (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_ECC_TYPE                     (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_INJECT_TYPE                  (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_ACCESSIBLE                   (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_ROW_WIDTH                    (32U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_SIZE                     (8192U)


#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID              (2U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_ECC_TYPE            (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_INJECT_TYPE         (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_ACCESSIBLE          (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_ROW_WIDTH           (32U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_SIZE            (16384U)


#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID              (3U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_ECC_TYPE            (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_INJECT_TYPE         (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_ACCESSIBLE          (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_ROW_WIDTH           (32U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_SIZE            (16384U)


#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID                         (4U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_ECC_TYPE                       (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_INJECT_TYPE                    (0U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_ACCESSIBLE                     (1U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_ROW_WIDTH                      (32U)
#define SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_SIZE                       (32768U)

/* Properties of ECC Aggregator instance : MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)


/* Properties of ECC Aggregator instance : MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)


/* Properties of ECC Aggregator instance : MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)


/* Properties of ECC Aggregator instance : MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)

/* Properties of ECC Aggregator instance : MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)

/* Properties of ECC Aggregator instance : MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)

/* Properties of ECC Aggregator instance : MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)

/* Properties of ECC Aggregator instance : MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR */
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS                                             (1U)


#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID                 (0U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ECC_TYPE               (0U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_INJECT_TYPE            (0U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ACCESSIBLE             (1U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_ROW_WIDTH              (32U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_SIZE               (17408U)

/* Properties of ECC Aggregator instance : FSS_OSPI_RAM_ECC_AGGR */
#define SDL_FSS_OSPI_RAM_ECC_AGGR_NUM_RAMS                                                         (1U)


#define SDL_FSS_OSPI_RAM_ECC_AGGR_RAM_ID                                                           (0U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR_ECC_TYPE                                                         (0U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR_INJECT_TYPE                                                      (0U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR_ACCESSIBLE                                                       (1U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR_ROW_WIDTH                                                        (32U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR_RAM_SIZE                                                         (256U)

/* Properties of ECC Aggregator instance : FSS_FOTA_8051_RAM_ECC_AGGR */
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_NUM_RAMS                                                    (1U)


#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_RAM_ID                                                      (0U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_ECC_TYPE                                                    (0U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_INJECT_TYPE                                                 (0U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_ACCESSIBLE                                                  (1U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_ROW_WIDTH                                                   (32U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR_RAM_SIZE                                                    (512U)

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
#define SDL_ECC_AGGR_NUM_ECC_AGGREGATORS                                                           (18U)

#ifdef __cplusplus
}
#endif
#endif /* SDLR_SOC_ECC_AGGR_H_ */
