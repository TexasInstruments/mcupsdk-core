/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CSLR_ECAP_H_
#define CSLR_ECAP_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t TSCTR;
    volatile uint32_t CTRPHS;
    volatile uint32_t CAP1;
    volatile uint32_t CAP2;
    volatile uint32_t CAP3;
    volatile uint32_t CAP4;
    volatile uint8_t  Resv_36[12];
    volatile uint32_t ECCTL0;
    volatile uint16_t ECCTL1;
    volatile uint16_t ECCTL2;
    volatile uint16_t ECEINT;
    volatile uint16_t ECFLG;
    volatile uint16_t ECCLR;
    volatile uint16_t ECFRC;
    volatile uint8_t  Resv_60[8];
    volatile uint32_t ECAPSYNCINSEL;
    volatile uint32_t HRCTL;
    volatile uint8_t  Resv_72[4];
    volatile uint32_t HRINTEN;
    volatile uint32_t HRFLG;
    volatile uint32_t HRCLR;
    volatile uint32_t HRFRC;
    volatile uint32_t HRCALPRD;
    volatile uint32_t HRSYSCLKCTR;
    volatile uint32_t HRSYSCLKCAP;
    volatile uint32_t HRCLKCTR;
    volatile uint32_t HRCLKCAP;
    volatile uint8_t  Resv_116[8];
    volatile uint32_t HRDEBUGCTL;
    volatile uint32_t HRDEBUGOBSERVE1;
    volatile uint32_t HRDEBUGOBSERVE2;
    volatile uint32_t MUNIT_COMMON_CTL;
    volatile uint8_t  Resv_192[60];
    volatile uint32_t MUNIT_1_CTL;
    volatile uint32_t MUNIT_1_SHADOW_CTL;
    volatile uint8_t  Resv_208[8];
    volatile uint32_t MUNIT_1_MIN;
    volatile uint32_t MUNIT_1_MAX;
    volatile uint32_t MUNIT_1_MIN_SHADOW;
    volatile uint32_t MUNIT_1_MAX_SHADOW;
    volatile uint32_t MUNIT_1_DEBUG_RANGE_MIN;
    volatile uint32_t MUNIT_1_DEBUG_RANGE_MAX;
    volatile uint8_t  Resv_256[24];
    volatile uint32_t MUNIT_2_CTL;
    volatile uint32_t MUNIT_2_SHADOW_CTL;
    volatile uint8_t  Resv_272[8];
    volatile uint32_t MUNIT_2_MIN;
    volatile uint32_t MUNIT_2_MAX;
    volatile uint32_t MUNIT_2_MIN_SHADOW;
    volatile uint32_t MUNIT_2_MAX_SHADOW;
    volatile uint32_t MUNIT_2_DEBUG_RANGE_MIN;
    volatile uint32_t MUNIT_2_DEBUG_RANGE_MAX;
} CSL_ecapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ECAP_TSCTR                                                         (0x00000000U)
#define CSL_ECAP_CTRPHS                                                        (0x00000004U)
#define CSL_ECAP_CAP1                                                          (0x00000008U)
#define CSL_ECAP_CAP2                                                          (0x0000000CU)
#define CSL_ECAP_CAP3                                                          (0x00000010U)
#define CSL_ECAP_CAP4                                                          (0x00000014U)
#define CSL_ECAP_ECCTL0                                                        (0x00000024U)
#define CSL_ECAP_ECCTL1                                                        (0x00000028U)
#define CSL_ECAP_ECCTL2                                                        (0x0000002AU)
#define CSL_ECAP_ECEINT                                                        (0x0000002CU)
#define CSL_ECAP_ECFLG                                                         (0x0000002EU)
#define CSL_ECAP_ECCLR                                                         (0x00000030U)
#define CSL_ECAP_ECFRC                                                         (0x00000032U)
#define CSL_ECAP_ECAPSYNCINSEL                                                 (0x0000003CU)
#define CSL_ECAP_HRCTL                                                         (0x00000040U)
#define CSL_ECAP_HRINTEN                                                       (0x00000048U)
#define CSL_ECAP_HRFLG                                                         (0x0000004CU)
#define CSL_ECAP_HRCLR                                                         (0x00000050U)
#define CSL_ECAP_HRFRC                                                         (0x00000054U)
#define CSL_ECAP_HRCALPRD                                                      (0x00000058U)
#define CSL_ECAP_HRSYSCLKCTR                                                   (0x0000005CU)
#define CSL_ECAP_HRSYSCLKCAP                                                   (0x00000060U)
#define CSL_ECAP_HRCLKCTR                                                      (0x00000064U)
#define CSL_ECAP_HRCLKCAP                                                      (0x00000068U)
#define CSL_ECAP_HRDEBUGCTL                                                    (0x00000074U)
#define CSL_ECAP_HRDEBUGOBSERVE1                                               (0x00000078U)
#define CSL_ECAP_HRDEBUGOBSERVE2                                               (0x0000007CU)
#define CSL_ECAP_MUNIT_COMMON_CTL                                              (0x00000080U)
#define CSL_ECAP_MUNIT_1_CTL                                                   (0x000000C0U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL                                            (0x000000C4U)
#define CSL_ECAP_MUNIT_1_MIN                                                   (0x000000D0U)
#define CSL_ECAP_MUNIT_1_MAX                                                   (0x000000D4U)
#define CSL_ECAP_MUNIT_1_MIN_SHADOW                                            (0x000000D8U)
#define CSL_ECAP_MUNIT_1_MAX_SHADOW                                            (0x000000DCU)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN                                       (0x000000E0U)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX                                       (0x000000E4U)
#define CSL_ECAP_MUNIT_2_CTL                                                   (0x00000100U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL                                            (0x00000104U)
#define CSL_ECAP_MUNIT_2_MIN                                                   (0x00000110U)
#define CSL_ECAP_MUNIT_2_MAX                                                   (0x00000114U)
#define CSL_ECAP_MUNIT_2_MIN_SHADOW                                            (0x00000118U)
#define CSL_ECAP_MUNIT_2_MAX_SHADOW                                            (0x0000011CU)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN                                       (0x00000120U)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX                                       (0x00000124U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TSCTR */

#define CSL_ECAP_TSCTR_TSCTR_MASK                                              (0xFFFFFFFFU)
#define CSL_ECAP_TSCTR_TSCTR_SHIFT                                             (0x00000000U)
#define CSL_ECAP_TSCTR_TSCTR_RESETVAL                                          (0x00000000U)
#define CSL_ECAP_TSCTR_TSCTR_MAX                                               (0xFFFFFFFFU)

#define CSL_ECAP_TSCTR_RESETVAL                                                (0x00000000U)

/* CTRPHS */

#define CSL_ECAP_CTRPHS_CTRPHS_MASK                                            (0xFFFFFFFFU)
#define CSL_ECAP_CTRPHS_CTRPHS_SHIFT                                           (0x00000000U)
#define CSL_ECAP_CTRPHS_CTRPHS_RESETVAL                                        (0x00000000U)
#define CSL_ECAP_CTRPHS_CTRPHS_MAX                                             (0xFFFFFFFFU)

#define CSL_ECAP_CTRPHS_RESETVAL                                               (0x00000000U)

/* CAP1 */

#define CSL_ECAP_CAP1_CAP1_MASK                                                (0xFFFFFFFFU)
#define CSL_ECAP_CAP1_CAP1_SHIFT                                               (0x00000000U)
#define CSL_ECAP_CAP1_CAP1_RESETVAL                                            (0x00000000U)
#define CSL_ECAP_CAP1_CAP1_MAX                                                 (0xFFFFFFFFU)

#define CSL_ECAP_CAP1_RESETVAL                                                 (0x00000000U)

/* CAP2 */

#define CSL_ECAP_CAP2_CAP2_MASK                                                (0xFFFFFFFFU)
#define CSL_ECAP_CAP2_CAP2_SHIFT                                               (0x00000000U)
#define CSL_ECAP_CAP2_CAP2_RESETVAL                                            (0x00000000U)
#define CSL_ECAP_CAP2_CAP2_MAX                                                 (0xFFFFFFFFU)

#define CSL_ECAP_CAP2_RESETVAL                                                 (0x00000000U)

/* CAP3 */

#define CSL_ECAP_CAP3_CAP3_MASK                                                (0xFFFFFFFFU)
#define CSL_ECAP_CAP3_CAP3_SHIFT                                               (0x00000000U)
#define CSL_ECAP_CAP3_CAP3_RESETVAL                                            (0x00000000U)
#define CSL_ECAP_CAP3_CAP3_MAX                                                 (0xFFFFFFFFU)

#define CSL_ECAP_CAP3_RESETVAL                                                 (0x00000000U)

/* CAP4 */

#define CSL_ECAP_CAP4_CAP4_MASK                                                (0xFFFFFFFFU)
#define CSL_ECAP_CAP4_CAP4_SHIFT                                               (0x00000000U)
#define CSL_ECAP_CAP4_CAP4_RESETVAL                                            (0x00000000U)
#define CSL_ECAP_CAP4_CAP4_MAX                                                 (0xFFFFFFFFU)

#define CSL_ECAP_CAP4_RESETVAL                                                 (0x00000000U)

/* ECCTL0 */

#define CSL_ECAP_ECCTL0_INPUTSEL_MASK                                          (0x000000FFU)
#define CSL_ECAP_ECCTL0_INPUTSEL_SHIFT                                         (0x00000000U)
#define CSL_ECAP_ECCTL0_INPUTSEL_RESETVAL                                      (0x000000FFU)
#define CSL_ECAP_ECCTL0_INPUTSEL_MAX                                           (0x000000FFU)

#define CSL_ECAP_ECCTL0_RESERVED_1_MASK                                        (0x00000F00U)
#define CSL_ECAP_ECCTL0_RESERVED_1_SHIFT                                       (0x00000008U)
#define CSL_ECAP_ECCTL0_RESERVED_1_RESETVAL                                    (0x00000000U)
#define CSL_ECAP_ECCTL0_RESERVED_1_MAX                                         (0x0000000FU)

#define CSL_ECAP_ECCTL0_QUALPRD_MASK                                           (0x0000F000U)
#define CSL_ECAP_ECCTL0_QUALPRD_SHIFT                                          (0x0000000CU)
#define CSL_ECAP_ECCTL0_QUALPRD_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_ECCTL0_QUALPRD_MAX                                            (0x0000000FU)

#define CSL_ECAP_ECCTL0_SOCEVTSEL_MASK                                         (0x00030000U)
#define CSL_ECAP_ECCTL0_SOCEVTSEL_SHIFT                                        (0x00000010U)
#define CSL_ECAP_ECCTL0_SOCEVTSEL_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_ECCTL0_SOCEVTSEL_MAX                                          (0x00000003U)

#define CSL_ECAP_ECCTL0_RESERVED_2_MASK                                        (0xFFFC0000U)
#define CSL_ECAP_ECCTL0_RESERVED_2_SHIFT                                       (0x00000012U)
#define CSL_ECAP_ECCTL0_RESERVED_2_RESETVAL                                    (0x00000000U)
#define CSL_ECAP_ECCTL0_RESERVED_2_MAX                                         (0x00003FFFU)

#define CSL_ECAP_ECCTL0_RESETVAL                                               (0x000000FFU)

/* ECCTL1 */

#define CSL_ECAP_ECCTL1_CAP1POL_MASK                                           (0x0001U)
#define CSL_ECAP_ECCTL1_CAP1POL_SHIFT                                          (0x0000U)
#define CSL_ECAP_ECCTL1_CAP1POL_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CAP1POL_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CAP1POL_VAL_ECAP_CAP_EVENT1_RISE                       (0x0U)
#define CSL_ECAP_ECCTL1_CAP1POL_VAL_ECAP_CAP_EVENT1_FALL                       (0x1U)

#define CSL_ECAP_ECCTL1_CTRRST1_MASK                                           (0x0002U)
#define CSL_ECAP_ECCTL1_CTRRST1_SHIFT                                          (0x0001U)
#define CSL_ECAP_ECCTL1_CTRRST1_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CTRRST1_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CTRRST1_VAL_ECAP_DO_NOT_RESET_EVENT1                   (0x0U)
#define CSL_ECAP_ECCTL1_CTRRST1_VAL_ECAP_RESET_EVENT1                          (0x1U)

#define CSL_ECAP_ECCTL1_CAP2POL_MASK                                           (0x0004U)
#define CSL_ECAP_ECCTL1_CAP2POL_SHIFT                                          (0x0002U)
#define CSL_ECAP_ECCTL1_CAP2POL_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CAP2POL_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CAP2POL_VAL_ECAP_CAP_EVENT2_RISE                       (0x0U)
#define CSL_ECAP_ECCTL1_CAP2POL_VAL_ECAP_CAP_EVENT2_FALL                       (0x1U)

#define CSL_ECAP_ECCTL1_CTRRST2_MASK                                           (0x0008U)
#define CSL_ECAP_ECCTL1_CTRRST2_SHIFT                                          (0x0003U)
#define CSL_ECAP_ECCTL1_CTRRST2_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CTRRST2_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CTRRST2_VAL_ECAP_DO_NOT_RESET_EVENT2                   (0x0U)
#define CSL_ECAP_ECCTL1_CTRRST2_VAL_ECAP_RESET_EVENT2                          (0x1U)

#define CSL_ECAP_ECCTL1_CAP3POL_MASK                                           (0x0010U)
#define CSL_ECAP_ECCTL1_CAP3POL_SHIFT                                          (0x0004U)
#define CSL_ECAP_ECCTL1_CAP3POL_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CAP3POL_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CAP3POL_VAL_ECAP_CAP_EVENT3_RISE                       (0x0U)
#define CSL_ECAP_ECCTL1_CAP3POL_VAL_ECAP_CAP_EVENT3_FALL                       (0x1U)

#define CSL_ECAP_ECCTL1_CTRRST3_MASK                                           (0x0020U)
#define CSL_ECAP_ECCTL1_CTRRST3_SHIFT                                          (0x0005U)
#define CSL_ECAP_ECCTL1_CTRRST3_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CTRRST3_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CTRRST3_VAL_ECAP_DO_NOT_RESET_EVENT3                   (0x0U)
#define CSL_ECAP_ECCTL1_CTRRST3_VAL_ECAP_RESET_EVENT3                          (0x1U)

#define CSL_ECAP_ECCTL1_CAP4POL_MASK                                           (0x0040U)
#define CSL_ECAP_ECCTL1_CAP4POL_SHIFT                                          (0x0006U)
#define CSL_ECAP_ECCTL1_CAP4POL_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CAP4POL_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CAP4POL_VAL_ECAP_CAP_EVENT4_RISE                       (0x0U)
#define CSL_ECAP_ECCTL1_CAP4POL_VAL_ECAP_CAP_EVENT4_FALL                       (0x1U)

#define CSL_ECAP_ECCTL1_CTRRST4_MASK                                           (0x0080U)
#define CSL_ECAP_ECCTL1_CTRRST4_SHIFT                                          (0x0007U)
#define CSL_ECAP_ECCTL1_CTRRST4_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CTRRST4_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CTRRST4_VAL_ECAP_DO_NOT_RESET_EVENT4                   (0x0U)
#define CSL_ECAP_ECCTL1_CTRRST4_VAL_ECAP_RESET_EVENT4                          (0x1U)

#define CSL_ECAP_ECCTL1_CAPLDEN_MASK                                           (0x0100U)
#define CSL_ECAP_ECCTL1_CAPLDEN_SHIFT                                          (0x0008U)
#define CSL_ECAP_ECCTL1_CAPLDEN_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL1_CAPLDEN_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL1_CAPLDEN_VAL_ECAP_DISABLE                               (0x0U)
#define CSL_ECAP_ECCTL1_CAPLDEN_VAL_ECAP_ENABLE                                (0x1U)

#define CSL_ECAP_ECCTL1_PRESCALE_MASK                                          (0x3E00U)
#define CSL_ECAP_ECCTL1_PRESCALE_SHIFT                                         (0x0009U)
#define CSL_ECAP_ECCTL1_PRESCALE_RESETVAL                                      (0x0000U)
#define CSL_ECAP_ECCTL1_PRESCALE_MAX                                           (0x001FU)

#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV1                                 (0x0U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV2                                 (0x1U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV4                                 (0x2U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV6                                 (0x3U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV8                                 (0x4U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV10                                (0x5U)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV60                                (0x1EU)
#define CSL_ECAP_ECCTL1_PRESCALE_VAL_ECAP_DIV62                                (0x1FU)

#define CSL_ECAP_ECCTL1_FREE_SOFT_MASK                                         (0xC000U)
#define CSL_ECAP_ECCTL1_FREE_SOFT_SHIFT                                        (0x000EU)
#define CSL_ECAP_ECCTL1_FREE_SOFT_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECCTL1_FREE_SOFT_MAX                                          (0x0003U)

#define CSL_ECAP_ECCTL1_FREE_SOFT_VAL_ECAP_STOP_EMU                            (0x0U)
#define CSL_ECAP_ECCTL1_FREE_SOFT_VAL_ECAP_RUNS_UNTIL                          (0x1U)
#define CSL_ECAP_ECCTL1_FREE_SOFT_VAL_ECAP_UNAF_EMU_SUS                        (0x2U)
#define CSL_ECAP_ECCTL1_FREE_SOFT_VAL_ECAP_UNAF_EMU_SUS2                       (0x3U)

#define CSL_ECAP_ECCTL1_RESETVAL                                               (0x0000U)

/* ECCTL2 */

#define CSL_ECAP_ECCTL2_CONT_ONESHT_MASK                                       (0x0001U)
#define CSL_ECAP_ECCTL2_CONT_ONESHT_SHIFT                                      (0x0000U)
#define CSL_ECAP_ECCTL2_CONT_ONESHT_RESETVAL                                   (0x0000U)
#define CSL_ECAP_ECCTL2_CONT_ONESHT_MAX                                        (0x0001U)

#define CSL_ECAP_ECCTL2_CONT_ONESHT_VAL_ECAP_OPP_CONT                          (0x0U)
#define CSL_ECAP_ECCTL2_CONT_ONESHT_VAL_ECAP_OPP_ONE                           (0x1U)

#define CSL_ECAP_ECCTL2_STOP_WRAP_MASK                                         (0x0006U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_SHIFT                                        (0x0001U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_RESETVAL                                     (0x0003U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_MAX                                          (0x0003U)

#define CSL_ECAP_ECCTL2_STOP_WRAP_VAL_ECAP_STOPEVENT1_WRAPEVENT2               (0x0U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_VAL_ECAP_STOPEVENT2_WRAPEVENT2               (0x1U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_VAL_ECAP_STOPEVENT3_WRAPEVENT2               (0x2U)
#define CSL_ECAP_ECCTL2_STOP_WRAP_VAL_ECAP_STOPEVENT4_WRAPEVENT2               (0x3U)

#define CSL_ECAP_ECCTL2_REARM_MASK                                             (0x0008U)
#define CSL_ECAP_ECCTL2_REARM_SHIFT                                            (0x0003U)
#define CSL_ECAP_ECCTL2_REARM_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECCTL2_REARM_MAX                                              (0x0001U)

#define CSL_ECAP_ECCTL2_REARM_VAL_ECAP_NO_EFFECT_RETURNS_0                     (0x0U)
#define CSL_ECAP_ECCTL2_REARM_VAL_ECAP_ARMS_ONESHOT                            (0x1U)

#define CSL_ECAP_ECCTL2_TSCTRSTOP_MASK                                         (0x0010U)
#define CSL_ECAP_ECCTL2_TSCTRSTOP_SHIFT                                        (0x0004U)
#define CSL_ECAP_ECCTL2_TSCTRSTOP_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECCTL2_TSCTRSTOP_MAX                                          (0x0001U)

#define CSL_ECAP_ECCTL2_TSCTRSTOP_VAL_ECAP_TSCTR_STOPPED                       (0x0U)
#define CSL_ECAP_ECCTL2_TSCTRSTOP_VAL_ECAP_TSCTR_FREE_RUNNING                  (0x1U)

#define CSL_ECAP_ECCTL2_SYNCI_EN_MASK                                          (0x0020U)
#define CSL_ECAP_ECCTL2_SYNCI_EN_SHIFT                                         (0x0005U)
#define CSL_ECAP_ECCTL2_SYNCI_EN_RESETVAL                                      (0x0000U)
#define CSL_ECAP_ECCTL2_SYNCI_EN_MAX                                           (0x0001U)

#define CSL_ECAP_ECCTL2_SYNCI_EN_VAL_ECAP_DISABLE_SYNC_IN                      (0x0U)
#define CSL_ECAP_ECCTL2_SYNCI_EN_VAL_ECAP_ENABLE_COUNTER_REGISTER              (0x1U)

#define CSL_ECAP_ECCTL2_SYNCO_SEL_MASK                                         (0x00C0U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_SHIFT                                        (0x0006U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_MAX                                          (0x0003U)

#define CSL_ECAP_ECCTL2_SYNCO_SEL_VAL_SWSYNC                                   (0x0U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_VAL_ECAP_CTR_PRD_TO_SYNCOUT                  (0x1U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_VAL_ECAP_DISABLE_SYNC_OUT                    (0x3U)
#define CSL_ECAP_ECCTL2_SYNCO_SEL_VAL_ECAP_DISABLE_SYNC_OUT                    (0x3U)

#define CSL_ECAP_ECCTL2_SWSYNC_MASK                                            (0x0100U)
#define CSL_ECAP_ECCTL2_SWSYNC_SHIFT                                           (0x0008U)
#define CSL_ECAP_ECCTL2_SWSYNC_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECCTL2_SWSYNC_MAX                                             (0x0001U)

#define CSL_ECAP_ECCTL2_SWSYNC_VAL_ECAP_ZERO_NOEFFECT                          (0x0U)
#define CSL_ECAP_ECCTL2_SWSYNC_VAL_ECAP_WRITE_TSCTR                            (0x1U)

#define CSL_ECAP_ECCTL2_CAP_APWM_MASK                                          (0x0200U)
#define CSL_ECAP_ECCTL2_CAP_APWM_SHIFT                                         (0x0009U)
#define CSL_ECAP_ECCTL2_CAP_APWM_RESETVAL                                      (0x0000U)
#define CSL_ECAP_ECCTL2_CAP_APWM_MAX                                           (0x0001U)

#define CSL_ECAP_ECCTL2_CAP_APWM_VAL_ECAP_MODULE                               (0x0U)
#define CSL_ECAP_ECCTL2_CAP_APWM_VAL_ECAP_MODULE_APWM                          (0x1U)

#define CSL_ECAP_ECCTL2_APWMPOL_MASK                                           (0x0400U)
#define CSL_ECAP_ECCTL2_APWMPOL_SHIFT                                          (0x000AU)
#define CSL_ECAP_ECCTL2_APWMPOL_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECCTL2_APWMPOL_MAX                                            (0x0001U)

#define CSL_ECAP_ECCTL2_APWMPOL_VAL_ECAP_OUTPUT_ACTIVE_HIGH                    (0x0U)
#define CSL_ECAP_ECCTL2_APWMPOL_VAL_ECAP_OUTPUT_ACTIVE_LOW                     (0x1U)

#define CSL_ECAP_ECCTL2_CTRFILTRESET_MASK                                      (0x0800U)
#define CSL_ECAP_ECCTL2_CTRFILTRESET_SHIFT                                     (0x000BU)
#define CSL_ECAP_ECCTL2_CTRFILTRESET_RESETVAL                                  (0x0000U)
#define CSL_ECAP_ECCTL2_CTRFILTRESET_MAX                                       (0x0001U)

#define CSL_ECAP_ECCTL2_DMAEVTSEL_MASK                                         (0x3000U)
#define CSL_ECAP_ECCTL2_DMAEVTSEL_SHIFT                                        (0x000CU)
#define CSL_ECAP_ECCTL2_DMAEVTSEL_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECCTL2_DMAEVTSEL_MAX                                          (0x0003U)

#define CSL_ECAP_ECCTL2_MODCNTRSTS_MASK                                        (0xC000U)
#define CSL_ECAP_ECCTL2_MODCNTRSTS_SHIFT                                       (0x000EU)
#define CSL_ECAP_ECCTL2_MODCNTRSTS_RESETVAL                                    (0x0000U)
#define CSL_ECAP_ECCTL2_MODCNTRSTS_MAX                                         (0x0003U)

#define CSL_ECAP_ECCTL2_RESETVAL                                               (0x0006U)

/* ECEINT */

#define CSL_ECAP_ECEINT_RESERVED_1_MASK                                        (0x0001U)
#define CSL_ECAP_ECEINT_RESERVED_1_SHIFT                                       (0x0000U)
#define CSL_ECAP_ECEINT_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_ECAP_ECEINT_RESERVED_1_MAX                                         (0x0001U)

#define CSL_ECAP_ECEINT_CEVT1_MASK                                             (0x0002U)
#define CSL_ECAP_ECEINT_CEVT1_SHIFT                                            (0x0001U)
#define CSL_ECAP_ECEINT_CEVT1_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECEINT_CEVT1_MAX                                              (0x0001U)

#define CSL_ECAP_ECEINT_CEVT1_VAL_ECAP_DISAB_CAP1_INTERRUPT                    (0x0U)
#define CSL_ECAP_ECEINT_CEVT1_VAL_ECAP_ENAB_CAP1_INTERRUPT                     (0x1U)

#define CSL_ECAP_ECEINT_CEVT2_MASK                                             (0x0004U)
#define CSL_ECAP_ECEINT_CEVT2_SHIFT                                            (0x0002U)
#define CSL_ECAP_ECEINT_CEVT2_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECEINT_CEVT2_MAX                                              (0x0001U)

#define CSL_ECAP_ECEINT_CEVT2_VAL_ECAP_DISAB_CAP2_INTERRUPT                    (0x0U)
#define CSL_ECAP_ECEINT_CEVT2_VAL_ECAP_ENAB_CAP2_INTERRUPT                     (0x1U)

#define CSL_ECAP_ECEINT_CEVT3_MASK                                             (0x0008U)
#define CSL_ECAP_ECEINT_CEVT3_SHIFT                                            (0x0003U)
#define CSL_ECAP_ECEINT_CEVT3_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECEINT_CEVT3_MAX                                              (0x0001U)

#define CSL_ECAP_ECEINT_CEVT3_VAL_ECAP_DISAB_CAP3_INTERRUPT                    (0x0U)
#define CSL_ECAP_ECEINT_CEVT3_VAL_ECAP_ENAB_CAP3_INTERRUPT                     (0x1U)

#define CSL_ECAP_ECEINT_CEVT4_MASK                                             (0x0010U)
#define CSL_ECAP_ECEINT_CEVT4_SHIFT                                            (0x0004U)
#define CSL_ECAP_ECEINT_CEVT4_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECEINT_CEVT4_MAX                                              (0x0001U)

#define CSL_ECAP_ECEINT_CEVT4_VAL_ECAP_DISAB_CAP4_INTERRUPT                    (0x0U)
#define CSL_ECAP_ECEINT_CEVT4_VAL_ECAP_ENAB_CAP4_INTERRUPT                     (0x1U)

#define CSL_ECAP_ECEINT_CTROVF_MASK                                            (0x0020U)
#define CSL_ECAP_ECEINT_CTROVF_SHIFT                                           (0x0005U)
#define CSL_ECAP_ECEINT_CTROVF_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECEINT_CTROVF_MAX                                             (0x0001U)

#define CSL_ECAP_ECEINT_CTROVF_VAL_ECAP_DISAB_CO_INTERRUPT                     (0x0U)
#define CSL_ECAP_ECEINT_CTROVF_VAL_ECAP_ENAB_CO_INTERRUPT                      (0x1U)

#define CSL_ECAP_ECEINT_CTR_EQ_PRD_MASK                                        (0x0040U)
#define CSL_ECAP_ECEINT_CTR_EQ_PRD_SHIFT                                       (0x0006U)
#define CSL_ECAP_ECEINT_CTR_EQ_PRD_RESETVAL                                    (0x0000U)
#define CSL_ECAP_ECEINT_CTR_EQ_PRD_MAX                                         (0x0001U)

#define CSL_ECAP_ECEINT_CTR_EQ_PRD_VAL_ECAP_DISAB_PE_INTERRUPT                 (0x0U)
#define CSL_ECAP_ECEINT_CTR_EQ_PRD_VAL_ECAP_ENAB_PE_INTERRUPT                  (0x1U)

#define CSL_ECAP_ECEINT_CTR_EQ_CMP_MASK                                        (0x0080U)
#define CSL_ECAP_ECEINT_CTR_EQ_CMP_SHIFT                                       (0x0007U)
#define CSL_ECAP_ECEINT_CTR_EQ_CMP_RESETVAL                                    (0x0000U)
#define CSL_ECAP_ECEINT_CTR_EQ_CMP_MAX                                         (0x0001U)

#define CSL_ECAP_ECEINT_CTR_EQ_CMP_VAL_ECAP_DISAB_CE_INTERRUPT                 (0x0U)
#define CSL_ECAP_ECEINT_CTR_EQ_CMP_VAL_ECAP_ENAB_CE_INTERRUPT                  (0x1U)

#define CSL_ECAP_ECEINT_HRERROR_MASK                                           (0x0100U)
#define CSL_ECAP_ECEINT_HRERROR_SHIFT                                          (0x0008U)
#define CSL_ECAP_ECEINT_HRERROR_RESETVAL                                       (0x0000U)
#define CSL_ECAP_ECEINT_HRERROR_MAX                                            (0x0001U)

#define CSL_ECAP_ECEINT_HRERROR_VAL_ECAP_DISAB_HRERROR_INTERRUPT               (0x0U)
#define CSL_ECAP_ECEINT_HRERROR_VAL_ECAP_ENAB_HRERROR_INTERRUPT                (0x1U)

#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT1_MASK                                (0x0200U)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT1_SHIFT                               (0x0009U)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT1_RESETVAL                            (0x0000U)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT1_MAX                                 (0x0001U)

#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT2_MASK                                (0x0400U)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT2_SHIFT                               (0x000AU)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT2_RESETVAL                            (0x0000U)
#define CSL_ECAP_ECEINT_MUNIT_1_ERROR_EVT2_MAX                                 (0x0001U)

#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT1_MASK                                (0x0800U)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT1_SHIFT                               (0x000BU)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT1_RESETVAL                            (0x0000U)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT1_MAX                                 (0x0001U)

#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT2_MASK                                (0x1000U)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT2_SHIFT                               (0x000CU)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT2_RESETVAL                            (0x0000U)
#define CSL_ECAP_ECEINT_MUNIT_2_ERROR_EVT2_MAX                                 (0x0001U)

#define CSL_ECAP_ECEINT_RESERVED_2_MASK                                        (0xE000U)
#define CSL_ECAP_ECEINT_RESERVED_2_SHIFT                                       (0x000DU)
#define CSL_ECAP_ECEINT_RESERVED_2_RESETVAL                                    (0x0000U)
#define CSL_ECAP_ECEINT_RESERVED_2_MAX                                         (0x0007U)

#define CSL_ECAP_ECEINT_RESETVAL                                               (0x0000U)

/* ECFLG */

#define CSL_ECAP_ECFLG_INT_MASK                                                (0x0001U)
#define CSL_ECAP_ECFLG_INT_SHIFT                                               (0x0000U)
#define CSL_ECAP_ECFLG_INT_RESETVAL                                            (0x0000U)
#define CSL_ECAP_ECFLG_INT_MAX                                                 (0x0001U)

#define CSL_ECAP_ECFLG_INT_VAL_ECAP_INDICATE_NO_EVENT                          (0x0U)
#define CSL_ECAP_ECFLG_INT_VAL_ECAP_INDICATE_INTERRUPT                         (0x1U)

#define CSL_ECAP_ECFLG_CEVT1_MASK                                              (0x0002U)
#define CSL_ECAP_ECFLG_CEVT1_SHIFT                                             (0x0001U)
#define CSL_ECAP_ECFLG_CEVT1_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFLG_CEVT1_MAX                                               (0x0001U)

#define CSL_ECAP_ECFLG_CEVT1_VAL_ECAP_INDICATE_NO_EVENT                        (0x0U)
#define CSL_ECAP_ECFLG_CEVT1_VAL_ECAP_INDICATE_1ST_EVENT_ECAPX                 (0x1U)

#define CSL_ECAP_ECFLG_CEVT2_MASK                                              (0x0004U)
#define CSL_ECAP_ECFLG_CEVT2_SHIFT                                             (0x0002U)
#define CSL_ECAP_ECFLG_CEVT2_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFLG_CEVT2_MAX                                               (0x0001U)

#define CSL_ECAP_ECFLG_CEVT2_VAL_ECAP_INDICATE_NO_EVENT                        (0x0U)
#define CSL_ECAP_ECFLG_CEVT2_VAL_ECAP_INDICATE_2ND_EVENT_ECAPX                 (0x1U)

#define CSL_ECAP_ECFLG_CEVT3_MASK                                              (0x0008U)
#define CSL_ECAP_ECFLG_CEVT3_SHIFT                                             (0x0003U)
#define CSL_ECAP_ECFLG_CEVT3_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFLG_CEVT3_MAX                                               (0x0001U)

#define CSL_ECAP_ECFLG_CEVT3_VAL_ECAP_INDICATE_NO_EVENT                        (0x0U)
#define CSL_ECAP_ECFLG_CEVT3_VAL_ECAP_INDICATE_3RD_EVENT_ECAPX                 (0x1U)

#define CSL_ECAP_ECFLG_CEVT4_MASK                                              (0x0010U)
#define CSL_ECAP_ECFLG_CEVT4_SHIFT                                             (0x0004U)
#define CSL_ECAP_ECFLG_CEVT4_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFLG_CEVT4_MAX                                               (0x0001U)

#define CSL_ECAP_ECFLG_CEVT4_VAL_ECAP_INDICATE_NO_EVENT                        (0x0U)
#define CSL_ECAP_ECFLG_CEVT4_VAL_ECAP_INDICATE_4TH_EVENT_ECAPX                 (0x1U)

#define CSL_ECAP_ECFLG_CTROVF_MASK                                             (0x0020U)
#define CSL_ECAP_ECFLG_CTROVF_SHIFT                                            (0x0005U)
#define CSL_ECAP_ECFLG_CTROVF_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECFLG_CTROVF_MAX                                              (0x0001U)

#define CSL_ECAP_ECFLG_CTROVF_VAL_ECAP_INDICATE_NO_EVENT                       (0x0U)
#define CSL_ECAP_ECFLG_CTROVF_VAL_ECAP_INDICATE_COUNTER_TRANS                  (0x1U)

#define CSL_ECAP_ECFLG_CTR_PRD_MASK                                            (0x0040U)
#define CSL_ECAP_ECFLG_CTR_PRD_SHIFT                                           (0x0006U)
#define CSL_ECAP_ECFLG_CTR_PRD_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFLG_CTR_PRD_MAX                                             (0x0001U)

#define CSL_ECAP_ECFLG_CTR_PRD_VAL_ECAP_INDICATE_NO_EVENT                      (0x0U)
#define CSL_ECAP_ECFLG_CTR_PRD_VAL_ECAP_INDICATE_PERIOD_VALUE_RESET            (0x1U)

#define CSL_ECAP_ECFLG_CTR_CMP_MASK                                            (0x0080U)
#define CSL_ECAP_ECFLG_CTR_CMP_SHIFT                                           (0x0007U)
#define CSL_ECAP_ECFLG_CTR_CMP_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFLG_CTR_CMP_MAX                                             (0x0001U)

#define CSL_ECAP_ECFLG_CTR_CMP_VAL_ECAP_INDICATE_NO_EVENT                      (0x0U)
#define CSL_ECAP_ECFLG_CTR_CMP_VAL_ECAP_INDICATE_COUNTER_COMPARE_REG           (0x1U)

#define CSL_ECAP_ECFLG_HRERROR_MASK                                            (0x0100U)
#define CSL_ECAP_ECFLG_HRERROR_SHIFT                                           (0x0008U)
#define CSL_ECAP_ECFLG_HRERROR_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFLG_HRERROR_MAX                                             (0x0001U)

#define CSL_ECAP_ECFLG_HRERROR_VAL_ECAP_INDICATE_NO_EVENT                      (0x0U)
#define CSL_ECAP_ECFLG_HRERROR_VAL_ECAP_INDICATE_HIGH_RESOLUTION_ERROR         (0x1U)

#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT1_MASK                                 (0x0200U)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT1_SHIFT                                (0x0009U)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT2_MASK                                 (0x0400U)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT2_SHIFT                                (0x000AU)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFLG_MUNIT_1_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT1_MASK                                 (0x0800U)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT1_SHIFT                                (0x000BU)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT2_MASK                                 (0x1000U)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT2_SHIFT                                (0x000CU)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFLG_MUNIT_2_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECFLG_RESERVED_1_MASK                                         (0xE000U)
#define CSL_ECAP_ECFLG_RESERVED_1_SHIFT                                        (0x000DU)
#define CSL_ECAP_ECFLG_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECFLG_RESERVED_1_MAX                                          (0x0007U)

#define CSL_ECAP_ECFLG_RESETVAL                                                (0x0000U)

/* ECCLR */

#define CSL_ECAP_ECCLR_INT_MASK                                                (0x0001U)
#define CSL_ECAP_ECCLR_INT_SHIFT                                               (0x0000U)
#define CSL_ECAP_ECCLR_INT_RESETVAL                                            (0x0000U)
#define CSL_ECAP_ECCLR_INT_MAX                                                 (0x0001U)

#define CSL_ECAP_ECCLR_INT_VAL_ECAP_0_NO_EFFECT                                (0x0U)
#define CSL_ECAP_ECCLR_INT_VAL_ECAP_1_CLEARS_INT                               (0x1U)

#define CSL_ECAP_ECCLR_CEVT1_MASK                                              (0x0002U)
#define CSL_ECAP_ECCLR_CEVT1_SHIFT                                             (0x0001U)
#define CSL_ECAP_ECCLR_CEVT1_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECCLR_CEVT1_MAX                                               (0x0001U)

#define CSL_ECAP_ECCLR_CEVT1_VAL_ECAP_0_NO_EFFECT                              (0x0U)
#define CSL_ECAP_ECCLR_CEVT1_VAL_ECAP_1_CLEARS_CEVT1                           (0x1U)

#define CSL_ECAP_ECCLR_CEVT2_MASK                                              (0x0004U)
#define CSL_ECAP_ECCLR_CEVT2_SHIFT                                             (0x0002U)
#define CSL_ECAP_ECCLR_CEVT2_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECCLR_CEVT2_MAX                                               (0x0001U)

#define CSL_ECAP_ECCLR_CEVT2_VAL_ECAP_0_NO_EFFECT                              (0x0U)
#define CSL_ECAP_ECCLR_CEVT2_VAL_ECAP_1_CLEARS_CEVT2                           (0x1U)

#define CSL_ECAP_ECCLR_CEVT3_MASK                                              (0x0008U)
#define CSL_ECAP_ECCLR_CEVT3_SHIFT                                             (0x0003U)
#define CSL_ECAP_ECCLR_CEVT3_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECCLR_CEVT3_MAX                                               (0x0001U)

#define CSL_ECAP_ECCLR_CEVT3_VAL_ECAP_0_NO_EFFECT                              (0x0U)
#define CSL_ECAP_ECCLR_CEVT3_VAL_ECAP_1_CLEARS_CEVT3                           (0x1U)

#define CSL_ECAP_ECCLR_CEVT4_MASK                                              (0x0010U)
#define CSL_ECAP_ECCLR_CEVT4_SHIFT                                             (0x0004U)
#define CSL_ECAP_ECCLR_CEVT4_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECCLR_CEVT4_MAX                                               (0x0001U)

#define CSL_ECAP_ECCLR_CEVT4_VAL_ECAP_0_NO_EFFECT                              (0x0U)
#define CSL_ECAP_ECCLR_CEVT4_VAL_ECAP_1_CLEARS_CEVT4                           (0x1U)

#define CSL_ECAP_ECCLR_CTROVF_MASK                                             (0x0020U)
#define CSL_ECAP_ECCLR_CTROVF_SHIFT                                            (0x0005U)
#define CSL_ECAP_ECCLR_CTROVF_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECCLR_CTROVF_MAX                                              (0x0001U)

#define CSL_ECAP_ECCLR_CTROVF_VAL_ECAP_0_NO_EFFECT                             (0x0U)
#define CSL_ECAP_ECCLR_CTROVF_VAL_ECAP_1_CLEARS_CTROVF                         (0x1U)

#define CSL_ECAP_ECCLR_CTR_PRD_MASK                                            (0x0040U)
#define CSL_ECAP_ECCLR_CTR_PRD_SHIFT                                           (0x0006U)
#define CSL_ECAP_ECCLR_CTR_PRD_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECCLR_CTR_PRD_MAX                                             (0x0001U)

#define CSL_ECAP_ECCLR_CTR_PRD_VAL_ECAP_0_NO_EFFECT                            (0x0U)
#define CSL_ECAP_ECCLR_CTR_PRD_VAL_ECAP_1_CLEARS_CTR_PRD                       (0x1U)

#define CSL_ECAP_ECCLR_CTR_CMP_MASK                                            (0x0080U)
#define CSL_ECAP_ECCLR_CTR_CMP_SHIFT                                           (0x0007U)
#define CSL_ECAP_ECCLR_CTR_CMP_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECCLR_CTR_CMP_MAX                                             (0x0001U)

#define CSL_ECAP_ECCLR_CTR_CMP_VAL_ECAP_0_NO_EFFECT                            (0x0U)
#define CSL_ECAP_ECCLR_CTR_CMP_VAL_ECAP_1_CLEARS_CTR_CMP                       (0x1U)

#define CSL_ECAP_ECCLR_HRERROR_MASK                                            (0x0100U)
#define CSL_ECAP_ECCLR_HRERROR_SHIFT                                           (0x0008U)
#define CSL_ECAP_ECCLR_HRERROR_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECCLR_HRERROR_MAX                                             (0x0001U)

#define CSL_ECAP_ECCLR_HRERROR_VAL_ECAP_0_NO_EFFECT                            (0x0U)
#define CSL_ECAP_ECCLR_HRERROR_VAL_ECAP_1_CLEARS_HRERROR                       (0x1U)

#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT1_MASK                                 (0x0200U)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT1_SHIFT                                (0x0009U)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT2_MASK                                 (0x0400U)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT2_SHIFT                                (0x000AU)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECCLR_MUNIT_1_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT1_MASK                                 (0x0800U)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT1_SHIFT                                (0x000BU)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT2_MASK                                 (0x1000U)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT2_SHIFT                                (0x000CU)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECCLR_MUNIT_2_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECCLR_RESERVED_1_MASK                                         (0xE000U)
#define CSL_ECAP_ECCLR_RESERVED_1_SHIFT                                        (0x000DU)
#define CSL_ECAP_ECCLR_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECCLR_RESERVED_1_MAX                                          (0x0007U)

#define CSL_ECAP_ECCLR_RESETVAL                                                (0x0000U)

/* ECFRC */

#define CSL_ECAP_ECFRC_RESERVED_1_MASK                                         (0x0001U)
#define CSL_ECAP_ECFRC_RESERVED_1_SHIFT                                        (0x0000U)
#define CSL_ECAP_ECFRC_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECFRC_RESERVED_1_MAX                                          (0x0001U)

#define CSL_ECAP_ECFRC_CEVT1_MASK                                              (0x0002U)
#define CSL_ECAP_ECFRC_CEVT1_SHIFT                                             (0x0001U)
#define CSL_ECAP_ECFRC_CEVT1_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFRC_CEVT1_MAX                                               (0x0001U)

#define CSL_ECAP_ECFRC_CEVT1_VAL_ECAP_NO_EFFECT_0                              (0x0U)
#define CSL_ECAP_ECFRC_CEVT1_VAL_ECAP_1_SETS_CEVT1                             (0x1U)

#define CSL_ECAP_ECFRC_CEVT2_MASK                                              (0x0004U)
#define CSL_ECAP_ECFRC_CEVT2_SHIFT                                             (0x0002U)
#define CSL_ECAP_ECFRC_CEVT2_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFRC_CEVT2_MAX                                               (0x0001U)

#define CSL_ECAP_ECFRC_CEVT2_VAL_ECAP_NO_EFFECT_0                              (0x0U)
#define CSL_ECAP_ECFRC_CEVT2_VAL_ECAP_1_SETS_CEVT2                             (0x1U)

#define CSL_ECAP_ECFRC_CEVT3_MASK                                              (0x0008U)
#define CSL_ECAP_ECFRC_CEVT3_SHIFT                                             (0x0003U)
#define CSL_ECAP_ECFRC_CEVT3_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFRC_CEVT3_MAX                                               (0x0001U)

#define CSL_ECAP_ECFRC_CEVT3_VAL_ECAP_NO_EFFECT_0                              (0x0U)
#define CSL_ECAP_ECFRC_CEVT3_VAL_ECAP_1_SETS_CEVT3                             (0x1U)

#define CSL_ECAP_ECFRC_CEVT4_MASK                                              (0x0010U)
#define CSL_ECAP_ECFRC_CEVT4_SHIFT                                             (0x0004U)
#define CSL_ECAP_ECFRC_CEVT4_RESETVAL                                          (0x0000U)
#define CSL_ECAP_ECFRC_CEVT4_MAX                                               (0x0001U)

#define CSL_ECAP_ECFRC_CEVT4_VAL_ECAP_NO_EFFECT_0                              (0x0U)
#define CSL_ECAP_ECFRC_CEVT4_VAL_ECAP_1_SETS_CEVT4                             (0x1U)

#define CSL_ECAP_ECFRC_CTROVF_MASK                                             (0x0020U)
#define CSL_ECAP_ECFRC_CTROVF_SHIFT                                            (0x0005U)
#define CSL_ECAP_ECFRC_CTROVF_RESETVAL                                         (0x0000U)
#define CSL_ECAP_ECFRC_CTROVF_MAX                                              (0x0001U)

#define CSL_ECAP_ECFRC_CTROVF_VAL_ECAP_NO_EFFECT_0                             (0x0U)
#define CSL_ECAP_ECFRC_CTROVF_VAL_ECAP_1_SETS_CTROVF                           (0x1U)

#define CSL_ECAP_ECFRC_CTR_PRD_MASK                                            (0x0040U)
#define CSL_ECAP_ECFRC_CTR_PRD_SHIFT                                           (0x0006U)
#define CSL_ECAP_ECFRC_CTR_PRD_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFRC_CTR_PRD_MAX                                             (0x0001U)

#define CSL_ECAP_ECFRC_CTR_PRD_VAL_ECAP_NO_EFFECT_0                            (0x0U)
#define CSL_ECAP_ECFRC_CTR_PRD_VAL_ECAP_1_CLEARS_CTR_PRD                       (0x1U)

#define CSL_ECAP_ECFRC_CTR_CMP_MASK                                            (0x0080U)
#define CSL_ECAP_ECFRC_CTR_CMP_SHIFT                                           (0x0007U)
#define CSL_ECAP_ECFRC_CTR_CMP_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFRC_CTR_CMP_MAX                                             (0x0001U)

#define CSL_ECAP_ECFRC_CTR_CMP_VAL_ECAP_NO_EFFECT_0                            (0x0U)
#define CSL_ECAP_ECFRC_CTR_CMP_VAL_ECAP_1_SETS_CTR_CMP                         (0x1U)

#define CSL_ECAP_ECFRC_HRERROR_MASK                                            (0x0100U)
#define CSL_ECAP_ECFRC_HRERROR_SHIFT                                           (0x0008U)
#define CSL_ECAP_ECFRC_HRERROR_RESETVAL                                        (0x0000U)
#define CSL_ECAP_ECFRC_HRERROR_MAX                                             (0x0001U)

#define CSL_ECAP_ECFRC_HRERROR_VAL_ECAP_NO_EFFECT_0                            (0x0U)
#define CSL_ECAP_ECFRC_HRERROR_VAL_ECAP_1_SETS_CTR_CMP                         (0x1U)

#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT1_MASK                                 (0x0200U)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT1_SHIFT                                (0x0009U)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT2_MASK                                 (0x0400U)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT2_SHIFT                                (0x000AU)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFRC_MUNIT_1_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT1_MASK                                 (0x0800U)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT1_SHIFT                                (0x000BU)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT1_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT1_MAX                                  (0x0001U)

#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT2_MASK                                 (0x1000U)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT2_SHIFT                                (0x000CU)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT2_RESETVAL                             (0x0000U)
#define CSL_ECAP_ECFRC_MUNIT_2_ERROR_EVT2_MAX                                  (0x0001U)

#define CSL_ECAP_ECFRC_RESERVED_2_MASK                                         (0xE000U)
#define CSL_ECAP_ECFRC_RESERVED_2_SHIFT                                        (0x000DU)
#define CSL_ECAP_ECFRC_RESERVED_2_RESETVAL                                     (0x0000U)
#define CSL_ECAP_ECFRC_RESERVED_2_MAX                                          (0x0007U)

#define CSL_ECAP_ECFRC_RESETVAL                                                (0x0000U)

/* ECAPSYNCINSEL */

#define CSL_ECAP_ECAPSYNCINSEL_SEL_MASK                                        (0x0000007FU)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_SHIFT                                       (0x00000000U)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_RESETVAL                                    (0x00000001U)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_MAX                                         (0x0000007FU)

#define CSL_ECAP_ECAPSYNCINSEL_RESERVED_1_MASK                                 (0xFFFFFF80U)
#define CSL_ECAP_ECAPSYNCINSEL_RESERVED_1_SHIFT                                (0x00000007U)
#define CSL_ECAP_ECAPSYNCINSEL_RESERVED_1_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECAPSYNCINSEL_RESERVED_1_MAX                                  (0x01FFFFFFU)

#define CSL_ECAP_ECAPSYNCINSEL_RESETVAL                                        (0x00000001U)

/* HRCTL */

#define CSL_ECAP_HRCTL_HRE_MASK                                                (0x00000001U)
#define CSL_ECAP_HRCTL_HRE_SHIFT                                               (0x00000000U)
#define CSL_ECAP_HRCTL_HRE_RESETVAL                                            (0x00000000U)
#define CSL_ECAP_HRCTL_HRE_MAX                                                 (0x00000001U)

#define CSL_ECAP_HRCTL_HRCLKE_MASK                                             (0x00000002U)
#define CSL_ECAP_HRCTL_HRCLKE_SHIFT                                            (0x00000001U)
#define CSL_ECAP_HRCTL_HRCLKE_RESETVAL                                         (0x00000000U)
#define CSL_ECAP_HRCTL_HRCLKE_MAX                                              (0x00000001U)

#define CSL_ECAP_HRCTL_PRDSEL_MASK                                             (0x00000004U)
#define CSL_ECAP_HRCTL_PRDSEL_SHIFT                                            (0x00000002U)
#define CSL_ECAP_HRCTL_PRDSEL_RESETVAL                                         (0x00000000U)
#define CSL_ECAP_HRCTL_PRDSEL_MAX                                              (0x00000001U)

#define CSL_ECAP_HRCTL_CALIBSTART_MASK                                         (0x00000008U)
#define CSL_ECAP_HRCTL_CALIBSTART_SHIFT                                        (0x00000003U)
#define CSL_ECAP_HRCTL_CALIBSTART_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRCTL_CALIBSTART_MAX                                          (0x00000001U)

#define CSL_ECAP_HRCTL_CALIBSTS_MASK                                           (0x00000010U)
#define CSL_ECAP_HRCTL_CALIBSTS_SHIFT                                          (0x00000004U)
#define CSL_ECAP_HRCTL_CALIBSTS_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_HRCTL_CALIBSTS_MAX                                            (0x00000001U)

#define CSL_ECAP_HRCTL_CALIBCONT_MASK                                          (0x00000020U)
#define CSL_ECAP_HRCTL_CALIBCONT_SHIFT                                         (0x00000005U)
#define CSL_ECAP_HRCTL_CALIBCONT_RESETVAL                                      (0x00000000U)
#define CSL_ECAP_HRCTL_CALIBCONT_MAX                                           (0x00000001U)

#define CSL_ECAP_HRCTL_RESERVED_1_MASK                                         (0xFFFFFFC0U)
#define CSL_ECAP_HRCTL_RESERVED_1_SHIFT                                        (0x00000006U)
#define CSL_ECAP_HRCTL_RESERVED_1_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRCTL_RESERVED_1_MAX                                          (0x03FFFFFFU)

#define CSL_ECAP_HRCTL_RESETVAL                                                (0x00000000U)

/* HRINTEN */

#define CSL_ECAP_HRINTEN_RESERVED_1_MASK                                       (0x00000001U)
#define CSL_ECAP_HRINTEN_RESERVED_1_SHIFT                                      (0x00000000U)
#define CSL_ECAP_HRINTEN_RESERVED_1_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_HRINTEN_RESERVED_1_MAX                                        (0x00000001U)

#define CSL_ECAP_HRINTEN_CALIBDONE_MASK                                        (0x00000002U)
#define CSL_ECAP_HRINTEN_CALIBDONE_SHIFT                                       (0x00000001U)
#define CSL_ECAP_HRINTEN_CALIBDONE_RESETVAL                                    (0x00000000U)
#define CSL_ECAP_HRINTEN_CALIBDONE_MAX                                         (0x00000001U)

#define CSL_ECAP_HRINTEN_CALPRDCHKSTS_MASK                                     (0x00000004U)
#define CSL_ECAP_HRINTEN_CALPRDCHKSTS_SHIFT                                    (0x00000002U)
#define CSL_ECAP_HRINTEN_CALPRDCHKSTS_RESETVAL                                 (0x00000000U)
#define CSL_ECAP_HRINTEN_CALPRDCHKSTS_MAX                                      (0x00000001U)

#define CSL_ECAP_HRINTEN_RESERVED_2_MASK                                       (0xFFFFFFF8U)
#define CSL_ECAP_HRINTEN_RESERVED_2_SHIFT                                      (0x00000003U)
#define CSL_ECAP_HRINTEN_RESERVED_2_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_HRINTEN_RESERVED_2_MAX                                        (0x1FFFFFFFU)

#define CSL_ECAP_HRINTEN_RESETVAL                                              (0x00000000U)

/* HRFLG */

#define CSL_ECAP_HRFLG_CALIBINT_MASK                                           (0x00000001U)
#define CSL_ECAP_HRFLG_CALIBINT_SHIFT                                          (0x00000000U)
#define CSL_ECAP_HRFLG_CALIBINT_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_HRFLG_CALIBINT_MAX                                            (0x00000001U)

#define CSL_ECAP_HRFLG_CALIBDONE_MASK                                          (0x00000002U)
#define CSL_ECAP_HRFLG_CALIBDONE_SHIFT                                         (0x00000001U)
#define CSL_ECAP_HRFLG_CALIBDONE_RESETVAL                                      (0x00000000U)
#define CSL_ECAP_HRFLG_CALIBDONE_MAX                                           (0x00000001U)

#define CSL_ECAP_HRFLG_CALPRDCHKSTS_MASK                                       (0x00000004U)
#define CSL_ECAP_HRFLG_CALPRDCHKSTS_SHIFT                                      (0x00000002U)
#define CSL_ECAP_HRFLG_CALPRDCHKSTS_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_HRFLG_CALPRDCHKSTS_MAX                                        (0x00000001U)

#define CSL_ECAP_HRFLG_RESERVED_1_MASK                                         (0xFFFFFFF8U)
#define CSL_ECAP_HRFLG_RESERVED_1_SHIFT                                        (0x00000003U)
#define CSL_ECAP_HRFLG_RESERVED_1_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRFLG_RESERVED_1_MAX                                          (0x1FFFFFFFU)

#define CSL_ECAP_HRFLG_RESETVAL                                                (0x00000000U)

/* HRCLR */

#define CSL_ECAP_HRCLR_CALIBINT_MASK                                           (0x00000001U)
#define CSL_ECAP_HRCLR_CALIBINT_SHIFT                                          (0x00000000U)
#define CSL_ECAP_HRCLR_CALIBINT_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_HRCLR_CALIBINT_MAX                                            (0x00000001U)

#define CSL_ECAP_HRCLR_CALIBDONE_MASK                                          (0x00000002U)
#define CSL_ECAP_HRCLR_CALIBDONE_SHIFT                                         (0x00000001U)
#define CSL_ECAP_HRCLR_CALIBDONE_RESETVAL                                      (0x00000000U)
#define CSL_ECAP_HRCLR_CALIBDONE_MAX                                           (0x00000001U)

#define CSL_ECAP_HRCLR_CALPRDCHKSTS_MASK                                       (0x00000004U)
#define CSL_ECAP_HRCLR_CALPRDCHKSTS_SHIFT                                      (0x00000002U)
#define CSL_ECAP_HRCLR_CALPRDCHKSTS_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_HRCLR_CALPRDCHKSTS_MAX                                        (0x00000001U)

#define CSL_ECAP_HRCLR_RESERVED_1_MASK                                         (0xFFFFFFF8U)
#define CSL_ECAP_HRCLR_RESERVED_1_SHIFT                                        (0x00000003U)
#define CSL_ECAP_HRCLR_RESERVED_1_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRCLR_RESERVED_1_MAX                                          (0x1FFFFFFFU)

#define CSL_ECAP_HRCLR_RESETVAL                                                (0x00000000U)

/* HRFRC */

#define CSL_ECAP_HRFRC_RESERVED_1_MASK                                         (0x00000001U)
#define CSL_ECAP_HRFRC_RESERVED_1_SHIFT                                        (0x00000000U)
#define CSL_ECAP_HRFRC_RESERVED_1_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRFRC_RESERVED_1_MAX                                          (0x00000001U)

#define CSL_ECAP_HRFRC_CALIBDONE_MASK                                          (0x00000002U)
#define CSL_ECAP_HRFRC_CALIBDONE_SHIFT                                         (0x00000001U)
#define CSL_ECAP_HRFRC_CALIBDONE_RESETVAL                                      (0x00000000U)
#define CSL_ECAP_HRFRC_CALIBDONE_MAX                                           (0x00000001U)

#define CSL_ECAP_HRFRC_CALPRDCHKSTS_MASK                                       (0x00000004U)
#define CSL_ECAP_HRFRC_CALPRDCHKSTS_SHIFT                                      (0x00000002U)
#define CSL_ECAP_HRFRC_CALPRDCHKSTS_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_HRFRC_CALPRDCHKSTS_MAX                                        (0x00000001U)

#define CSL_ECAP_HRFRC_RESERVED_2_MASK                                         (0xFFFFFFF8U)
#define CSL_ECAP_HRFRC_RESERVED_2_SHIFT                                        (0x00000003U)
#define CSL_ECAP_HRFRC_RESERVED_2_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_HRFRC_RESERVED_2_MAX                                          (0x1FFFFFFFU)

#define CSL_ECAP_HRFRC_RESETVAL                                                (0x00000000U)

/* HRCALPRD */

#define CSL_ECAP_HRCALPRD_PRD_MASK                                             (0xFFFFFFFFU)
#define CSL_ECAP_HRCALPRD_PRD_SHIFT                                            (0x00000000U)
#define CSL_ECAP_HRCALPRD_PRD_RESETVAL                                         (0x003FFFFFU)
#define CSL_ECAP_HRCALPRD_PRD_MAX                                              (0xFFFFFFFFU)

#define CSL_ECAP_HRCALPRD_RESETVAL                                             (0x003FFFFFU)

/* HRSYSCLKCTR */

#define CSL_ECAP_HRSYSCLKCTR_HRSYSCLKCTR_MASK                                  (0xFFFFFFFFU)
#define CSL_ECAP_HRSYSCLKCTR_HRSYSCLKCTR_SHIFT                                 (0x00000000U)
#define CSL_ECAP_HRSYSCLKCTR_HRSYSCLKCTR_RESETVAL                              (0x00000000U)
#define CSL_ECAP_HRSYSCLKCTR_HRSYSCLKCTR_MAX                                   (0xFFFFFFFFU)

#define CSL_ECAP_HRSYSCLKCTR_RESETVAL                                          (0x00000000U)

/* HRSYSCLKCAP */

#define CSL_ECAP_HRSYSCLKCAP_HRSYSCLKCAP_MASK                                  (0xFFFFFFFFU)
#define CSL_ECAP_HRSYSCLKCAP_HRSYSCLKCAP_SHIFT                                 (0x00000000U)
#define CSL_ECAP_HRSYSCLKCAP_HRSYSCLKCAP_RESETVAL                              (0x00000000U)
#define CSL_ECAP_HRSYSCLKCAP_HRSYSCLKCAP_MAX                                   (0xFFFFFFFFU)

#define CSL_ECAP_HRSYSCLKCAP_RESETVAL                                          (0x00000000U)

/* HRCLKCTR */

#define CSL_ECAP_HRCLKCTR_HRCLKCTR_MASK                                        (0xFFFFFFFFU)
#define CSL_ECAP_HRCLKCTR_HRCLKCTR_SHIFT                                       (0x00000000U)
#define CSL_ECAP_HRCLKCTR_HRCLKCTR_RESETVAL                                    (0x00000000U)
#define CSL_ECAP_HRCLKCTR_HRCLKCTR_MAX                                         (0xFFFFFFFFU)

#define CSL_ECAP_HRCLKCTR_RESETVAL                                             (0x00000000U)

/* HRCLKCAP */

#define CSL_ECAP_HRCLKCAP_HRCLKCAP_MASK                                        (0xFFFFFFFFU)
#define CSL_ECAP_HRCLKCAP_HRCLKCAP_SHIFT                                       (0x00000000U)
#define CSL_ECAP_HRCLKCAP_HRCLKCAP_RESETVAL                                    (0x00000000U)
#define CSL_ECAP_HRCLKCAP_HRCLKCAP_MAX                                         (0xFFFFFFFFU)

#define CSL_ECAP_HRCLKCAP_RESETVAL                                             (0x00000000U)

/* HRDEBUGCTL */

#define CSL_ECAP_HRDEBUGCTL_DISABLEINVSEL_MASK                                 (0x00000001U)
#define CSL_ECAP_HRDEBUGCTL_DISABLEINVSEL_SHIFT                                (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_DISABLEINVSEL_RESETVAL                             (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_DISABLEINVSEL_MAX                                  (0x00000001U)

#define CSL_ECAP_HRDEBUGCTL_DELAYRESETDLINE_MASK                               (0x00000002U)
#define CSL_ECAP_HRDEBUGCTL_DELAYRESETDLINE_SHIFT                              (0x00000001U)
#define CSL_ECAP_HRDEBUGCTL_DELAYRESETDLINE_RESETVAL                           (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_DELAYRESETDLINE_MAX                                (0x00000001U)

#define CSL_ECAP_HRDEBUGCTL_CAPIN_MMAP_SOURCE_MASK                             (0x00000004U)
#define CSL_ECAP_HRDEBUGCTL_CAPIN_MMAP_SOURCE_SHIFT                            (0x00000002U)
#define CSL_ECAP_HRDEBUGCTL_CAPIN_MMAP_SOURCE_RESETVAL                         (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_CAPIN_MMAP_SOURCE_MAX                              (0x00000001U)

#define CSL_ECAP_HRDEBUGCTL_RESERVED_1_MASK                                    (0x00000008U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_1_SHIFT                                   (0x00000003U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_1_MAX                                     (0x00000001U)

#define CSL_ECAP_HRDEBUGCTL_CALIB_INPUT_SEL_MASK                               (0x00000030U)
#define CSL_ECAP_HRDEBUGCTL_CALIB_INPUT_SEL_SHIFT                              (0x00000004U)
#define CSL_ECAP_HRDEBUGCTL_CALIB_INPUT_SEL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_CALIB_INPUT_SEL_MAX                                (0x00000003U)

#define CSL_ECAP_HRDEBUGCTL_RESERVED_2_MASK                                    (0x000000C0U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_2_SHIFT                                   (0x00000006U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_2_MAX                                     (0x00000003U)

#define CSL_ECAP_HRDEBUGCTL_OBSERVE_SRC_SEL_MASK                               (0x00000F00U)
#define CSL_ECAP_HRDEBUGCTL_OBSERVE_SRC_SEL_SHIFT                              (0x00000008U)
#define CSL_ECAP_HRDEBUGCTL_OBSERVE_SRC_SEL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_OBSERVE_SRC_SEL_MAX                                (0x0000000FU)

#define CSL_ECAP_HRDEBUGCTL_RESERVED_3_MASK                                    (0xFFFFF000U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_3_SHIFT                                   (0x0000000CU)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_3_RESETVAL                                (0x00000000U)
#define CSL_ECAP_HRDEBUGCTL_RESERVED_3_MAX                                     (0x000FFFFFU)

#define CSL_ECAP_HRDEBUGCTL_RESETVAL                                           (0x00000000U)

/* HRDEBUGOBSERVE1 */

#define CSL_ECAP_HRDEBUGOBSERVE1_HROUTH_MASK                                   (0xFFFFFFFFU)
#define CSL_ECAP_HRDEBUGOBSERVE1_HROUTH_SHIFT                                  (0x00000000U)
#define CSL_ECAP_HRDEBUGOBSERVE1_HROUTH_RESETVAL                               (0x00000000U)
#define CSL_ECAP_HRDEBUGOBSERVE1_HROUTH_MAX                                    (0xFFFFFFFFU)

#define CSL_ECAP_HRDEBUGOBSERVE1_RESETVAL                                      (0x00000000U)

/* HRDEBUGOBSERVE2 */

#define CSL_ECAP_HRDEBUGOBSERVE2_HROUTL_MASK                                   (0xFFFFFFFFU)
#define CSL_ECAP_HRDEBUGOBSERVE2_HROUTL_SHIFT                                  (0x00000000U)
#define CSL_ECAP_HRDEBUGOBSERVE2_HROUTL_RESETVAL                               (0x00000000U)
#define CSL_ECAP_HRDEBUGOBSERVE2_HROUTL_MAX                                    (0xFFFFFFFFU)

#define CSL_ECAP_HRDEBUGOBSERVE2_RESETVAL                                      (0x00000000U)

/* MUNIT_COMMON_CTL */

#define CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_MASK                                 (0x0000007FU)
#define CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_SHIFT                                (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_RESETVAL                             (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_TRIPSEL_MAX                                  (0x0000007FU)

#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_1_MASK                              (0x00000080U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_1_SHIFT                             (0x00000007U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_1_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_1_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_MASK                              (0x00007F00U)
#define CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_SHIFT                             (0x00000008U)
#define CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_GLDSTRBSEL_MAX                               (0x0000007FU)

#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_2_MASK                              (0x00008000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_2_SHIFT                             (0x0000000FU)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_2_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_2_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_3_MASK                              (0xFFFF0000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_3_SHIFT                             (0x00000010U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_3_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_COMMON_CTL_RESERVED_3_MAX                               (0x0000FFFFU)

#define CSL_ECAP_MUNIT_COMMON_CTL_RESETVAL                                     (0x00000000U)

/* MUNIT_1_CTL */

#define CSL_ECAP_MUNIT_1_CTL_EN_MASK                                           (0x00000001U)
#define CSL_ECAP_MUNIT_1_CTL_EN_SHIFT                                          (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_EN_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_EN_MAX                                            (0x00000001U)

#define CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_MASK                               (0x00000002U)
#define CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_SHIFT                              (0x00000001U)
#define CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_RESETVAL                           (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_DEBUG_RANGE_EN_MAX                                (0x00000001U)

#define CSL_ECAP_MUNIT_1_CTL_RESERVED_1_MASK                                   (0x000000FCU)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_1_SHIFT                                  (0x00000002U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_1_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_1_MAX                                    (0x0000003FU)

#define CSL_ECAP_MUNIT_1_CTL_MON_SEL_MASK                                      (0x00000F00U)
#define CSL_ECAP_MUNIT_1_CTL_MON_SEL_SHIFT                                     (0x00000008U)
#define CSL_ECAP_MUNIT_1_CTL_MON_SEL_RESETVAL                                  (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_MON_SEL_MAX                                       (0x0000000FU)

#define CSL_ECAP_MUNIT_1_CTL_RESERVED_2_MASK                                   (0x0000F000U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_2_SHIFT                                  (0x0000000CU)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_2_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_2_MAX                                    (0x0000000FU)

#define CSL_ECAP_MUNIT_1_CTL_RESERVED_3_MASK                                   (0xFFFF0000U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_3_SHIFT                                  (0x00000010U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_3_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_1_CTL_RESERVED_3_MAX                                    (0x0000FFFFU)

#define CSL_ECAP_MUNIT_1_CTL_RESETVAL                                          (0x00000000U)

/* MUNIT_1_SHADOW_CTL */

#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_MASK                              (0x00000001U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_SHIFT                             (0x00000000U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SYNCI_EN_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SWSYNC_MASK                                (0x00000002U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SWSYNC_SHIFT                               (0x00000001U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SWSYNC_RESETVAL                            (0x00000000U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_SWSYNC_MAX                                 (0x00000001U)

#define CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_MASK                              (0x00000004U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_SHIFT                             (0x00000002U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_LOADMODE_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_1_SHADOW_CTL_RESERVED_1_MASK                            (0xFFFFFFF8U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_RESERVED_1_SHIFT                           (0x00000003U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ECAP_MUNIT_1_SHADOW_CTL_RESERVED_1_MAX                             (0x1FFFFFFFU)

#define CSL_ECAP_MUNIT_1_SHADOW_CTL_RESETVAL                                   (0x00000000U)

/* MUNIT_1_MIN */

#define CSL_ECAP_MUNIT_1_MIN_MIN_VALUE_MASK                                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_MIN_MIN_VALUE_SHIFT                                   (0x00000000U)
#define CSL_ECAP_MUNIT_1_MIN_MIN_VALUE_RESETVAL                                (0x00000000U)
#define CSL_ECAP_MUNIT_1_MIN_MIN_VALUE_MAX                                     (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_MIN_RESETVAL                                          (0x00000000U)

/* MUNIT_1_MAX */

#define CSL_ECAP_MUNIT_1_MAX_MAX_VALUE_MASK                                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_MAX_MAX_VALUE_SHIFT                                   (0x00000000U)
#define CSL_ECAP_MUNIT_1_MAX_MAX_VALUE_RESETVAL                                (0x00000000U)
#define CSL_ECAP_MUNIT_1_MAX_MAX_VALUE_MAX                                     (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_MAX_RESETVAL                                          (0x00000000U)

/* MUNIT_1_MIN_SHADOW */

#define CSL_ECAP_MUNIT_1_MIN_SHADOW_MIN_VALUE_SHADOW_MASK                      (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_MIN_SHADOW_MIN_VALUE_SHADOW_SHIFT                     (0x00000000U)
#define CSL_ECAP_MUNIT_1_MIN_SHADOW_MIN_VALUE_SHADOW_RESETVAL                  (0x00000000U)
#define CSL_ECAP_MUNIT_1_MIN_SHADOW_MIN_VALUE_SHADOW_MAX                       (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_MIN_SHADOW_RESETVAL                                   (0x00000000U)

/* MUNIT_1_MAX_SHADOW */

#define CSL_ECAP_MUNIT_1_MAX_SHADOW_MAX_VALUE_SHADOW_MASK                      (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_MAX_SHADOW_MAX_VALUE_SHADOW_SHIFT                     (0x00000000U)
#define CSL_ECAP_MUNIT_1_MAX_SHADOW_MAX_VALUE_SHADOW_RESETVAL                  (0x00000000U)
#define CSL_ECAP_MUNIT_1_MAX_SHADOW_MAX_VALUE_SHADOW_MAX                       (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_MAX_SHADOW_RESETVAL                                   (0x00000000U)

/* MUNIT_1_DEBUG_RANGE_MIN */

#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN_MIN_VALUE_MASK                        (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN_MIN_VALUE_SHIFT                       (0x00000000U)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN_MIN_VALUE_RESETVAL                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN_MIN_VALUE_MAX                         (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MIN_RESETVAL                              (0xFFFFFFFFU)

/* MUNIT_1_DEBUG_RANGE_MAX */

#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX_MAX_VALUE_MASK                        (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX_MAX_VALUE_SHIFT                       (0x00000000U)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX_MAX_VALUE_RESETVAL                    (0x00000000U)
#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX_MAX_VALUE_MAX                         (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_1_DEBUG_RANGE_MAX_RESETVAL                              (0x00000000U)

/* MUNIT_2_CTL */

#define CSL_ECAP_MUNIT_2_CTL_EN_MASK                                           (0x00000001U)
#define CSL_ECAP_MUNIT_2_CTL_EN_SHIFT                                          (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_EN_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_EN_MAX                                            (0x00000001U)

#define CSL_ECAP_MUNIT_2_CTL_DEBUG_RANGE_EN_MASK                               (0x00000002U)
#define CSL_ECAP_MUNIT_2_CTL_DEBUG_RANGE_EN_SHIFT                              (0x00000001U)
#define CSL_ECAP_MUNIT_2_CTL_DEBUG_RANGE_EN_RESETVAL                           (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_DEBUG_RANGE_EN_MAX                                (0x00000001U)

#define CSL_ECAP_MUNIT_2_CTL_RESERVED_1_MASK                                   (0x000000FCU)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_1_SHIFT                                  (0x00000002U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_1_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_1_MAX                                    (0x0000003FU)

#define CSL_ECAP_MUNIT_2_CTL_MON_SEL_MASK                                      (0x00000F00U)
#define CSL_ECAP_MUNIT_2_CTL_MON_SEL_SHIFT                                     (0x00000008U)
#define CSL_ECAP_MUNIT_2_CTL_MON_SEL_RESETVAL                                  (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_MON_SEL_MAX                                       (0x0000000FU)

#define CSL_ECAP_MUNIT_2_CTL_RESERVED_2_MASK                                   (0x0000F000U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_2_SHIFT                                  (0x0000000CU)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_2_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_2_MAX                                    (0x0000000FU)

#define CSL_ECAP_MUNIT_2_CTL_RESERVED_3_MASK                                   (0xFFFF0000U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_3_SHIFT                                  (0x00000010U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_3_RESETVAL                               (0x00000000U)
#define CSL_ECAP_MUNIT_2_CTL_RESERVED_3_MAX                                    (0x0000FFFFU)

#define CSL_ECAP_MUNIT_2_CTL_RESETVAL                                          (0x00000000U)

/* MUNIT_2_SHADOW_CTL */

#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SYNCI_EN_MASK                              (0x00000001U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SYNCI_EN_SHIFT                             (0x00000000U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SYNCI_EN_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SYNCI_EN_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SWSYNC_MASK                                (0x00000002U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SWSYNC_SHIFT                               (0x00000001U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SWSYNC_RESETVAL                            (0x00000000U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_SWSYNC_MAX                                 (0x00000001U)

#define CSL_ECAP_MUNIT_2_SHADOW_CTL_LOADMODE_MASK                              (0x00000004U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_LOADMODE_SHIFT                             (0x00000002U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_LOADMODE_RESETVAL                          (0x00000000U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_LOADMODE_MAX                               (0x00000001U)

#define CSL_ECAP_MUNIT_2_SHADOW_CTL_RESERVED_1_MASK                            (0xFFFFFFF8U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_RESERVED_1_SHIFT                           (0x00000003U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ECAP_MUNIT_2_SHADOW_CTL_RESERVED_1_MAX                             (0x1FFFFFFFU)

#define CSL_ECAP_MUNIT_2_SHADOW_CTL_RESETVAL                                   (0x00000000U)

/* MUNIT_2_MIN */

#define CSL_ECAP_MUNIT_2_MIN_MIN_VALUE_MASK                                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_MIN_MIN_VALUE_SHIFT                                   (0x00000000U)
#define CSL_ECAP_MUNIT_2_MIN_MIN_VALUE_RESETVAL                                (0x00000000U)
#define CSL_ECAP_MUNIT_2_MIN_MIN_VALUE_MAX                                     (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_MIN_RESETVAL                                          (0x00000000U)

/* MUNIT_2_MAX */

#define CSL_ECAP_MUNIT_2_MAX_MAX_VALUE_MASK                                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_MAX_MAX_VALUE_SHIFT                                   (0x00000000U)
#define CSL_ECAP_MUNIT_2_MAX_MAX_VALUE_RESETVAL                                (0x00000000U)
#define CSL_ECAP_MUNIT_2_MAX_MAX_VALUE_MAX                                     (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_MAX_RESETVAL                                          (0x00000000U)

/* MUNIT_2_MIN_SHADOW */

#define CSL_ECAP_MUNIT_2_MIN_SHADOW_MIN_VALUE_SHADOW_MASK                      (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_MIN_SHADOW_MIN_VALUE_SHADOW_SHIFT                     (0x00000000U)
#define CSL_ECAP_MUNIT_2_MIN_SHADOW_MIN_VALUE_SHADOW_RESETVAL                  (0x00000000U)
#define CSL_ECAP_MUNIT_2_MIN_SHADOW_MIN_VALUE_SHADOW_MAX                       (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_MIN_SHADOW_RESETVAL                                   (0x00000000U)

/* MUNIT_2_MAX_SHADOW */

#define CSL_ECAP_MUNIT_2_MAX_SHADOW_MAX_VALUE_SHADOW_MASK                      (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_MAX_SHADOW_MAX_VALUE_SHADOW_SHIFT                     (0x00000000U)
#define CSL_ECAP_MUNIT_2_MAX_SHADOW_MAX_VALUE_SHADOW_RESETVAL                  (0x00000000U)
#define CSL_ECAP_MUNIT_2_MAX_SHADOW_MAX_VALUE_SHADOW_MAX                       (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_MAX_SHADOW_RESETVAL                                   (0x00000000U)

/* MUNIT_2_DEBUG_RANGE_MIN */

#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN_MIN_VALUE_MASK                        (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN_MIN_VALUE_SHIFT                       (0x00000000U)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN_MIN_VALUE_RESETVAL                    (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN_MIN_VALUE_MAX                         (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MIN_RESETVAL                              (0xFFFFFFFFU)

/* MUNIT_2_DEBUG_RANGE_MAX */

#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX_MAX_VALUE_MASK                        (0xFFFFFFFFU)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX_MAX_VALUE_SHIFT                       (0x00000000U)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX_MAX_VALUE_RESETVAL                    (0x00000000U)
#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX_MAX_VALUE_MAX                         (0xFFFFFFFFU)

#define CSL_ECAP_MUNIT_2_DEBUG_RANGE_MAX_RESETVAL                              (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
