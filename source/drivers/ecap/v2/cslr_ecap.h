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
* Register Overlay Structure __ALL__
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
    volatile uint32_t ECCTL1_ECCTL2;
    volatile uint32_t ECEINT_ECFLG;
    volatile uint32_t ECCLR_ECFRC;
    volatile uint8_t  Resv_60[8];
    volatile uint32_t ECAPSYNCINSEL;
} CSL_ecapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ECAP_TSCTR                                                    (0x00000000U)
#define CSL_ECAP_CTRPHS                                                   (0x00000004U)
#define CSL_ECAP_CAP1                                                     (0x00000008U)
#define CSL_ECAP_CAP2                                                     (0x0000000CU)
#define CSL_ECAP_CAP3                                                     (0x00000010U)
#define CSL_ECAP_CAP4                                                     (0x00000014U)
#define CSL_ECAP_ECCTL0                                                   (0x00000024U)
#define CSL_ECAP_ECCTL1_ECCTL2                                            (0x00000028U)
#define CSL_ECAP_ECEINT_ECFLG                                             (0x0000002CU)
#define CSL_ECAP_ECCLR_ECFRC                                              (0x00000030U)
#define CSL_ECAP_ECAPSYNCINSEL                                            (0x0000003CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TSCTR */

#define CSL_ECAP_TSCTR_TSCTR_MASK                                         (0xFFFFFFFFU)
#define CSL_ECAP_TSCTR_TSCTR_SHIFT                                        (0x00000000U)
#define CSL_ECAP_TSCTR_TSCTR_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_TSCTR_TSCTR_MAX                                          (0xFFFFFFFFU)

#define CSL_ECAP_TSCTR_RESETVAL                                           (0x00000000U)

/* CTRPHS */

#define CSL_ECAP_CTRPHS_CTRPHS_MASK                                       (0xFFFFFFFFU)
#define CSL_ECAP_CTRPHS_CTRPHS_SHIFT                                      (0x00000000U)
#define CSL_ECAP_CTRPHS_CTRPHS_RESETVAL                                   (0x00000000U)
#define CSL_ECAP_CTRPHS_CTRPHS_MAX                                        (0xFFFFFFFFU)

#define CSL_ECAP_CTRPHS_RESETVAL                                          (0x00000000U)

/* CAP1 */

#define CSL_ECAP_CAP1_CAP1_MASK                                           (0xFFFFFFFFU)
#define CSL_ECAP_CAP1_CAP1_SHIFT                                          (0x00000000U)
#define CSL_ECAP_CAP1_CAP1_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_CAP1_CAP1_MAX                                            (0xFFFFFFFFU)

#define CSL_ECAP_CAP1_RESETVAL                                            (0x00000000U)

/* CAP2 */

#define CSL_ECAP_CAP2_CAP2_MASK                                           (0xFFFFFFFFU)
#define CSL_ECAP_CAP2_CAP2_SHIFT                                          (0x00000000U)
#define CSL_ECAP_CAP2_CAP2_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_CAP2_CAP2_MAX                                            (0xFFFFFFFFU)

#define CSL_ECAP_CAP2_RESETVAL                                            (0x00000000U)

/* CAP3 */

#define CSL_ECAP_CAP3_CAP3_MASK                                           (0xFFFFFFFFU)
#define CSL_ECAP_CAP3_CAP3_SHIFT                                          (0x00000000U)
#define CSL_ECAP_CAP3_CAP3_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_CAP3_CAP3_MAX                                            (0xFFFFFFFFU)

#define CSL_ECAP_CAP3_RESETVAL                                            (0x00000000U)

/* CAP4 */

#define CSL_ECAP_CAP4_CAP4_MASK                                           (0xFFFFFFFFU)
#define CSL_ECAP_CAP4_CAP4_SHIFT                                          (0x00000000U)
#define CSL_ECAP_CAP4_CAP4_RESETVAL                                       (0x00000000U)
#define CSL_ECAP_CAP4_CAP4_MAX                                            (0xFFFFFFFFU)

#define CSL_ECAP_CAP4_RESETVAL                                            (0x00000000U)

/* ECCTL0 */

#define CSL_ECAP_ECCTL0_INPUTSEL_MASK                                     (0x0000007FU)
#define CSL_ECAP_ECCTL0_INPUTSEL_SHIFT                                    (0x00000000U)
#define CSL_ECAP_ECCTL0_INPUTSEL_RESETVAL                                 (0x0000007FU)
#define CSL_ECAP_ECCTL0_INPUTSEL_MAX                                      (0x0000007FU)

#define CSL_ECAP_ECCTL0_NU_1_MASK                                         (0xFFFFFF80U)
#define CSL_ECAP_ECCTL0_NU_1_SHIFT                                        (0x00000007U)
#define CSL_ECAP_ECCTL0_NU_1_RESETVAL                                     (0x00000000U)
#define CSL_ECAP_ECCTL0_NU_1_MAX                                          (0x01FFFFFFU)

#define CSL_ECAP_ECCTL0_RESETVAL                                          (0x0000007FU)

/* ECCTL1_ECCTL2 */

#define CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_MASK                               (0x00000001U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_SHIFT                              (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP1POL_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_MASK                               (0x00000002U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_SHIFT                              (0x00000001U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST1_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_MASK                               (0x00000004U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_SHIFT                              (0x00000002U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP2POL_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_MASK                               (0x00000008U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_SHIFT                              (0x00000003U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST2_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_MASK                               (0x00000010U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_SHIFT                              (0x00000004U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP3POL_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_MASK                               (0x00000020U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_SHIFT                              (0x00000005U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST3_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_MASK                               (0x00000040U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_SHIFT                              (0x00000006U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP4POL_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_MASK                               (0x00000080U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_SHIFT                              (0x00000007U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRRST4_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_MASK                               (0x00000100U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_SHIFT                              (0x00000008U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAPLDEN_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_PRESCALE_MASK                              (0x00003E00U)
#define CSL_ECAP_ECCTL1_ECCTL2_PRESCALE_SHIFT                             (0x00000009U)
#define CSL_ECAP_ECCTL1_ECCTL2_PRESCALE_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_PRESCALE_MAX                               (0x0000001FU)

#define CSL_ECAP_ECCTL1_ECCTL2_FREE_SOFT_MASK                             (0x0000C000U)
#define CSL_ECAP_ECCTL1_ECCTL2_FREE_SOFT_SHIFT                            (0x0000000EU)
#define CSL_ECAP_ECCTL1_ECCTL2_FREE_SOFT_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_FREE_SOFT_MAX                              (0x00000003U)

#define CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_MASK                           (0x00010000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_SHIFT                          (0x00000010U)
#define CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_RESETVAL                       (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CONT_ONESHT_MAX                            (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_STOP_WRAP_MASK                             (0x00060000U)
#define CSL_ECAP_ECCTL1_ECCTL2_STOP_WRAP_SHIFT                            (0x00000011U)
#define CSL_ECAP_ECCTL1_ECCTL2_STOP_WRAP_RESETVAL                         (0x00000003U)
#define CSL_ECAP_ECCTL1_ECCTL2_STOP_WRAP_MAX                              (0x00000003U)

#define CSL_ECAP_ECCTL1_ECCTL2_REARM_MASK                                 (0x00080000U)
#define CSL_ECAP_ECCTL1_ECCTL2_REARM_SHIFT                                (0x00000013U)
#define CSL_ECAP_ECCTL1_ECCTL2_REARM_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_REARM_MAX                                  (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_TSCTRSTOP_MASK                             (0x00100000U)
#define CSL_ECAP_ECCTL1_ECCTL2_TSCTRSTOP_SHIFT                            (0x00000014U)
#define CSL_ECAP_ECCTL1_ECCTL2_TSCTRSTOP_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_TSCTRSTOP_MAX                              (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_SYNCI_EN_MASK                              (0x00200000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCI_EN_SHIFT                             (0x00000015U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCI_EN_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCI_EN_MAX                               (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_SYNCO_SEL_MASK                             (0x00C00000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCO_SEL_SHIFT                            (0x00000016U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCO_SEL_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SYNCO_SEL_MAX                              (0x00000003U)

#define CSL_ECAP_ECCTL1_ECCTL2_SWSYNC_MASK                                (0x01000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SWSYNC_SHIFT                               (0x00000018U)
#define CSL_ECAP_ECCTL1_ECCTL2_SWSYNC_RESETVAL                            (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_SWSYNC_MAX                                 (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CAP_APWM_MASK                              (0x02000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP_APWM_SHIFT                             (0x00000019U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP_APWM_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CAP_APWM_MAX                               (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_APWMPOL_MASK                               (0x04000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_APWMPOL_SHIFT                              (0x0000001AU)
#define CSL_ECAP_ECCTL1_ECCTL2_APWMPOL_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_APWMPOL_MAX                                (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_CTRFILTRESET_MASK                          (0x08000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRFILTRESET_SHIFT                         (0x0000001BU)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRFILTRESET_RESETVAL                      (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_CTRFILTRESET_MAX                           (0x00000001U)

#define CSL_ECAP_ECCTL1_ECCTL2_DMAEVTSEL_MASK                             (0x30000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_DMAEVTSEL_SHIFT                            (0x0000001CU)
#define CSL_ECAP_ECCTL1_ECCTL2_DMAEVTSEL_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_DMAEVTSEL_MAX                              (0x00000003U)

#define CSL_ECAP_ECCTL1_ECCTL2_MODCNTRSTS_MASK                            (0xC0000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_MODCNTRSTS_SHIFT                           (0x0000001EU)
#define CSL_ECAP_ECCTL1_ECCTL2_MODCNTRSTS_RESETVAL                        (0x00000000U)
#define CSL_ECAP_ECCTL1_ECCTL2_MODCNTRSTS_MAX                             (0x00000003U)

#define CSL_ECAP_ECCTL1_ECCTL2_RESETVAL                                   (0x00060000U)

/* ECEINT_ECFLG */

#define CSL_ECAP_ECEINT_ECFLG_NU_2_MASK                                   (0x00000001U)
#define CSL_ECAP_ECEINT_ECFLG_NU_2_SHIFT                                  (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_NU_2_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_NU_2_MAX                                    (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT1_MASK                                  (0x00000002U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_SHIFT                                 (0x00000001U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_MAX                                   (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT2_MASK                                  (0x00000004U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_SHIFT                                 (0x00000002U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_MAX                                   (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT3_MASK                                  (0x00000008U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_SHIFT                                 (0x00000003U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_MAX                                   (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT4_MASK                                  (0x00000010U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_SHIFT                                 (0x00000004U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_MAX                                   (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTROVF_MASK                                 (0x00000020U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_SHIFT                                (0x00000005U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_MAX                                  (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_PRD_MASK                             (0x00000040U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_PRD_SHIFT                            (0x00000006U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_PRD_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_PRD_MAX                              (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_CMP_MASK                             (0x00000080U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_CMP_SHIFT                            (0x00000007U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_CMP_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_EQ_CMP_MAX                              (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_HRERROR_MASK                                (0x00000100U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_SHIFT                               (0x00000008U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_RESETVAL                            (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_MAX                                 (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_NU_3_MASK                                   (0x0000FE00U)
#define CSL_ECAP_ECEINT_ECFLG_NU_3_SHIFT                                  (0x00000009U)
#define CSL_ECAP_ECEINT_ECFLG_NU_3_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_NU_3_MAX                                    (0x0000007FU)

#define CSL_ECAP_ECEINT_ECFLG_INT_FLG_MASK                                (0x00010000U)
#define CSL_ECAP_ECEINT_ECFLG_INT_FLG_SHIFT                               (0x00000010U)
#define CSL_ECAP_ECEINT_ECFLG_INT_FLG_RESETVAL                            (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_INT_FLG_MAX                                 (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT1_FLG_MASK                              (0x00020000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_FLG_SHIFT                             (0x00000011U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_FLG_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT1_FLG_MAX                               (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT2_FLG_MASK                              (0x00040000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_FLG_SHIFT                             (0x00000012U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_FLG_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT2_FLG_MAX                               (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT3_FLG_MASK                              (0x00080000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_FLG_SHIFT                             (0x00000013U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_FLG_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT3_FLG_MAX                               (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CEVT4_FLG_MASK                              (0x00100000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_FLG_SHIFT                             (0x00000014U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_FLG_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CEVT4_FLG_MAX                               (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTROVF_FLG_MASK                             (0x00200000U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_FLG_SHIFT                            (0x00000015U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_FLG_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTROVF_FLG_MAX                              (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTR_PRD_FLG_MASK                            (0x00400000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_PRD_FLG_SHIFT                           (0x00000016U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_PRD_FLG_RESETVAL                        (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_PRD_FLG_MAX                             (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_CTR_CMP_FLG_MASK                            (0x00800000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_CMP_FLG_SHIFT                           (0x00000017U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_CMP_FLG_RESETVAL                        (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_CTR_CMP_FLG_MAX                             (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_HRERROR_FLG_MASK                            (0x01000000U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_FLG_SHIFT                           (0x00000018U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_FLG_RESETVAL                        (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_HRERROR_FLG_MAX                             (0x00000001U)

#define CSL_ECAP_ECEINT_ECFLG_NU_4_MASK                                   (0xFE000000U)
#define CSL_ECAP_ECEINT_ECFLG_NU_4_SHIFT                                  (0x00000019U)
#define CSL_ECAP_ECEINT_ECFLG_NU_4_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECEINT_ECFLG_NU_4_MAX                                    (0x0000007FU)

#define CSL_ECAP_ECEINT_ECFLG_RESETVAL                                    (0x00000000U)

/* ECCLR_ECFRC */

#define CSL_ECAP_ECCLR_ECFRC_INT_MASK                                     (0x00000001U)
#define CSL_ECAP_ECCLR_ECFRC_INT_SHIFT                                    (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_INT_RESETVAL                                 (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_INT_MAX                                      (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT1_MASK                                   (0x00000002U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_SHIFT                                  (0x00000001U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_MAX                                    (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT2_MASK                                   (0x00000004U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_SHIFT                                  (0x00000002U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_MAX                                    (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT3_MASK                                   (0x00000008U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_SHIFT                                  (0x00000003U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_MAX                                    (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT4_MASK                                   (0x00000010U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_SHIFT                                  (0x00000004U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_RESETVAL                               (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_MAX                                    (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTROVF_MASK                                  (0x00000020U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_SHIFT                                 (0x00000005U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_MAX                                   (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_MASK                                 (0x00000040U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_SHIFT                                (0x00000006U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_MAX                                  (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_MASK                                 (0x00000080U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_SHIFT                                (0x00000007U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_MAX                                  (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_HRERROR_MASK                                 (0x00000100U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_SHIFT                                (0x00000008U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_RESETVAL                             (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_MAX                                  (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_NU_5_MASK                                    (0x0001FE00U)
#define CSL_ECAP_ECCLR_ECFRC_NU_5_SHIFT                                   (0x00000009U)
#define CSL_ECAP_ECCLR_ECFRC_NU_5_RESETVAL                                (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_NU_5_MAX                                     (0x000000FFU)

#define CSL_ECAP_ECCLR_ECFRC_CEVT1_FRC_MASK                               (0x00020000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_FRC_SHIFT                              (0x00000011U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_FRC_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT1_FRC_MAX                                (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT2_FRC_MASK                               (0x00040000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_FRC_SHIFT                              (0x00000012U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_FRC_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT2_FRC_MAX                                (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT3_FRC_MASK                               (0x00080000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_FRC_SHIFT                              (0x00000013U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_FRC_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT3_FRC_MAX                                (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CEVT4_FRC_MASK                               (0x00100000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_FRC_SHIFT                              (0x00000014U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_FRC_RESETVAL                           (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CEVT4_FRC_MAX                                (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTROVF_FRC_MASK                              (0x00200000U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_FRC_SHIFT                             (0x00000015U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_FRC_RESETVAL                          (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTROVF_FRC_MAX                               (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_FRC_MASK                             (0x00400000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_FRC_SHIFT                            (0x00000016U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_FRC_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_PRD_FRC_MAX                              (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_FRC_MASK                             (0x00800000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_FRC_SHIFT                            (0x00000017U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_FRC_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_CTR_CMP_FRC_MAX                              (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_HRERROR_FRC_MASK                             (0x01000000U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_FRC_SHIFT                            (0x00000018U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_FRC_RESETVAL                         (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_HRERROR_FRC_MAX                              (0x00000001U)

#define CSL_ECAP_ECCLR_ECFRC_NU_6_MASK                                    (0xFE000000U)
#define CSL_ECAP_ECCLR_ECFRC_NU_6_SHIFT                                   (0x00000019U)
#define CSL_ECAP_ECCLR_ECFRC_NU_6_RESETVAL                                (0x00000000U)
#define CSL_ECAP_ECCLR_ECFRC_NU_6_MAX                                     (0x0000007FU)

#define CSL_ECAP_ECCLR_ECFRC_RESETVAL                                     (0x00000000U)

/* ECAPSYNCINSEL */

#define CSL_ECAP_ECAPSYNCINSEL_SEL_MASK                                   (0x0000001FU)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_SHIFT                                  (0x00000000U)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_RESETVAL                               (0x00000001U)
#define CSL_ECAP_ECAPSYNCINSEL_SEL_MAX                                    (0x0000001FU)

#define CSL_ECAP_ECAPSYNCINSEL_NU_7_MASK                                  (0xFFFFFFE0U)
#define CSL_ECAP_ECAPSYNCINSEL_NU_7_SHIFT                                 (0x00000005U)
#define CSL_ECAP_ECAPSYNCINSEL_NU_7_RESETVAL                              (0x00000000U)
#define CSL_ECAP_ECAPSYNCINSEL_NU_7_MAX                                   (0x07FFFFFFU)

#define CSL_ECAP_ECAPSYNCINSEL_RESETVAL                                   (0x00000001U)


#ifdef __cplusplus
}
#endif
#endif
