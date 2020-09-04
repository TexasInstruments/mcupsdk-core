/********************************************************************
 * Copyright (C) 2013-2014 Texas Instruments Incorporated.
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
#ifndef CSLR_ICSSECAP_H_
#define CSLR_ICSSECAP_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile Uint32 TSCNT;
    volatile Uint32 CNTPHS;
    volatile Uint32 CAP1;
    volatile Uint32 CAP2;
    volatile Uint32 CAP3;
    volatile Uint32 CAP4;
    volatile Uint8  RSVD0[14];
    volatile Uint16 ECCTL1;
    volatile Uint16 ECCTL2;
    volatile Uint16 ECEINT;
    volatile Uint16 ECFLG;
    volatile Uint8  RSVD1[2];
    volatile Uint16 ECCLR;
    volatile Uint8  RSVD2[2];
    volatile Uint16 ECFRC;
    volatile Uint8  RSVD3[38];
    volatile Uint32 PID;
    volatile Uint8  RSVD4[276];
} CSL_IcssEcapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* TIME STAMP COUNTER REGISTER */
#define CSL_ICSSECAP_TSCNT                                      (0x0U)

/* COUNTER PHASE CONTROL REGISTER */
#define CSL_ICSSECAP_CNTPHS                                     (0x4U)

/* CAPTURE-1 REGISTER */
#define CSL_ICSSECAP_CAP1                                       (0x8U)

/* CAPTURE-2 REGISTER */
#define CSL_ICSSECAP_CAP2                                       (0xCU)

/* CAPTURE-3 REGISTER */
#define CSL_ICSSECAP_CAP3                                       (0x10U)

/* CAPTURE-4 REGISTER */
#define CSL_ICSSECAP_CAP4                                       (0x14U)

/* ECAP CONTROL REGISTER 1 */
#define CSL_ICSSECAP_ECCTL1                                     (0x26U)

/* ECAP CONTROL REGISTER 2 */
#define CSL_ICSSECAP_ECCTL2                                     (0x28U)

/* ECAP INTERRUPT ENABLE REGISTER */
#define CSL_ICSSECAP_ECEINT                                     (0x2AU)

/* ECAP INTERRUPT FLAG REGISTER */
#define CSL_ICSSECAP_ECFLG                                      (0x2CU)

/* ECAP INTERRUPT CLEAR REGISTER */
#define CSL_ICSSECAP_ECCLR                                      (0x30U)

/* ECAP INTERRUPT FORCING REGISTER */
#define CSL_ICSSECAP_ECFRC                                      (0x34U)

/* ECAP PERIPHERAL ID REGISTER */
#define CSL_ICSSECAP_PID                                        (0x5CU)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* TSCNT */

#define CSL_ICSSECAP_TSCNT_TSCNT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSECAP_TSCNT_TSCNT_SHIFT                          (0U)
#define CSL_ICSSECAP_TSCNT_TSCNT_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_TSCNT_TSCNT_MAX                            (0xffffffffU)

#define CSL_ICSSECAP_TSCNT_RESETVAL                             (0x00000000U)

/* CNTPHS */

#define CSL_ICSSECAP_CNTPHS_CNTPHS_MASK                         (0xFFFFFFFFU)
#define CSL_ICSSECAP_CNTPHS_CNTPHS_SHIFT                        (0U)
#define CSL_ICSSECAP_CNTPHS_CNTPHS_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_CNTPHS_CNTPHS_MAX                          (0xffffffffU)

#define CSL_ICSSECAP_CNTPHS_RESETVAL                            (0x00000000U)

/* CAP1 */

#define CSL_ICSSECAP_CAP1_CAP1_MASK                             (0xFFFFFFFFU)
#define CSL_ICSSECAP_CAP1_CAP1_SHIFT                            (0U)
#define CSL_ICSSECAP_CAP1_CAP1_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_CAP1_CAP1_MAX                              (0xffffffffU)

#define CSL_ICSSECAP_CAP1_RESETVAL                              (0x00000000U)

/* CAP2 */

#define CSL_ICSSECAP_CAP2_CAP2_MASK                             (0xFFFFFFFFU)
#define CSL_ICSSECAP_CAP2_CAP2_SHIFT                            (0U)
#define CSL_ICSSECAP_CAP2_CAP2_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_CAP2_CAP2_MAX                              (0xffffffffU)

#define CSL_ICSSECAP_CAP2_RESETVAL                              (0x00000000U)

/* CAP3 */

#define CSL_ICSSECAP_CAP3_CAP3_MASK                             (0xFFFFFFFFU)
#define CSL_ICSSECAP_CAP3_CAP3_SHIFT                            (0U)
#define CSL_ICSSECAP_CAP3_CAP3_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_CAP3_CAP3_MAX                              (0xffffffffU)

#define CSL_ICSSECAP_CAP3_RESETVAL                              (0x00000000U)

/* CAP4 */

#define CSL_ICSSECAP_CAP4_CAP4_MASK                             (0xFFFFFFFFU)
#define CSL_ICSSECAP_CAP4_CAP4_SHIFT                            (0U)
#define CSL_ICSSECAP_CAP4_CAP4_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_CAP4_CAP4_MAX                              (0xffffffffU)

#define CSL_ICSSECAP_CAP4_RESETVAL                              (0x00000000U)

/* ECCTL1 */

#define CSL_ICSSECAP_ECCTL1_CAP1POL_MASK                        (0x00000001U)
#define CSL_ICSSECAP_ECCTL1_CAP1POL_SHIFT                       (0U)
#define CSL_ICSSECAP_ECCTL1_CAP1POL_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CAP1POL_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CTRRST1_MASK                        (0x00000002U)
#define CSL_ICSSECAP_ECCTL1_CTRRST1_SHIFT                       (1U)
#define CSL_ICSSECAP_ECCTL1_CTRRST1_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CTRRST1_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CAP2POL_MASK                        (0x00000004U)
#define CSL_ICSSECAP_ECCTL1_CAP2POL_SHIFT                       (2U)
#define CSL_ICSSECAP_ECCTL1_CAP2POL_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CAP2POL_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CTRRST2_MASK                        (0x00000008U)
#define CSL_ICSSECAP_ECCTL1_CTRRST2_SHIFT                       (3U)
#define CSL_ICSSECAP_ECCTL1_CTRRST2_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CTRRST2_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CAP3POL_MASK                        (0x00000010U)
#define CSL_ICSSECAP_ECCTL1_CAP3POL_SHIFT                       (4U)
#define CSL_ICSSECAP_ECCTL1_CAP3POL_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CAP3POL_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CTRRST3_MASK                        (0x00000020U)
#define CSL_ICSSECAP_ECCTL1_CTRRST3_SHIFT                       (5U)
#define CSL_ICSSECAP_ECCTL1_CTRRST3_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CTRRST3_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CAP4POL_MASK                        (0x00000040U)
#define CSL_ICSSECAP_ECCTL1_CAP4POL_SHIFT                       (6U)
#define CSL_ICSSECAP_ECCTL1_CAP4POL_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CAP4POL_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CTRRST4_MASK                        (0x00000080U)
#define CSL_ICSSECAP_ECCTL1_CTRRST4_SHIFT                       (7U)
#define CSL_ICSSECAP_ECCTL1_CTRRST4_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CTRRST4_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_CAPLDEN_MASK                        (0x00000100U)
#define CSL_ICSSECAP_ECCTL1_CAPLDEN_SHIFT                       (8U)
#define CSL_ICSSECAP_ECCTL1_CAPLDEN_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_CAPLDEN_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_EVTFLTPS_MASK                       (0x00003E00U)
#define CSL_ICSSECAP_ECCTL1_EVTFLTPS_SHIFT                      (9U)
#define CSL_ICSSECAP_ECCTL1_EVTFLTPS_RESETVAL                   (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_EVTFLTPS_MAX                        (0x0000001fU)

#define CSL_ICSSECAP_ECCTL1_SOFT_MASK                           (0x00004000U)
#define CSL_ICSSECAP_ECCTL1_SOFT_SHIFT                          (14U)
#define CSL_ICSSECAP_ECCTL1_SOFT_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_SOFT_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_FREE_MASK                           (0x00008000U)
#define CSL_ICSSECAP_ECCTL1_FREE_SHIFT                          (15U)
#define CSL_ICSSECAP_ECCTL1_FREE_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCTL1_FREE_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCTL1_RESETVAL                            (0x00000000U)

/* ECCTL2 */

#define CSL_ICSSECAP_ECCTL2_CONT_ONESHT_MASK                    (0x00000001U)
#define CSL_ICSSECAP_ECCTL2_CONT_ONESHT_SHIFT                   (0U)
#define CSL_ICSSECAP_ECCTL2_CONT_ONESHT_RESETVAL                (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_CONT_ONESHT_MAX                     (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_STOPVALUE_MASK                      (0x00000006U)
#define CSL_ICSSECAP_ECCTL2_STOPVALUE_SHIFT                     (1U)
#define CSL_ICSSECAP_ECCTL2_STOPVALUE_RESETVAL                  (0x00000003U)
#define CSL_ICSSECAP_ECCTL2_STOPVALUE_MAX                       (0x00000003U)

#define CSL_ICSSECAP_ECCTL2_REARM_RESET_MASK                    (0x00000008U)
#define CSL_ICSSECAP_ECCTL2_REARM_RESET_SHIFT                   (3U)
#define CSL_ICSSECAP_ECCTL2_REARM_RESET_RESETVAL                (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_REARM_RESET_MAX                     (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_TSCNTSTP_MASK                       (0x00000010U)
#define CSL_ICSSECAP_ECCTL2_TSCNTSTP_SHIFT                      (4U)
#define CSL_ICSSECAP_ECCTL2_TSCNTSTP_RESETVAL                   (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_TSCNTSTP_MAX                        (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_SYNCI_EN_MASK                       (0x00000020U)
#define CSL_ICSSECAP_ECCTL2_SYNCI_EN_SHIFT                      (5U)
#define CSL_ICSSECAP_ECCTL2_SYNCI_EN_RESETVAL                   (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_SYNCI_EN_MAX                        (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_SYNCO_SEL_MASK                      (0x000000C0U)
#define CSL_ICSSECAP_ECCTL2_SYNCO_SEL_SHIFT                     (6U)
#define CSL_ICSSECAP_ECCTL2_SYNCO_SEL_RESETVAL                  (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_SYNCO_SEL_MAX                       (0x00000003U)

#define CSL_ICSSECAP_ECCTL2_SWSYNC_MASK                         (0x00000100U)
#define CSL_ICSSECAP_ECCTL2_SWSYNC_SHIFT                        (8U)
#define CSL_ICSSECAP_ECCTL2_SWSYNC_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_SWSYNC_MAX                          (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_CAP_APWM_MASK                       (0x00000200U)
#define CSL_ICSSECAP_ECCTL2_CAP_APWM_SHIFT                      (9U)
#define CSL_ICSSECAP_ECCTL2_CAP_APWM_RESETVAL                   (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_CAP_APWM_MAX                        (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_APWMPOL_MASK                        (0x00000400U)
#define CSL_ICSSECAP_ECCTL2_APWMPOL_SHIFT                       (10U)
#define CSL_ICSSECAP_ECCTL2_APWMPOL_RESETVAL                    (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_APWMPOL_MAX                         (0x00000001U)

#define CSL_ICSSECAP_ECCTL2_FILTER_MASK                         (0x0000F800U)
#define CSL_ICSSECAP_ECCTL2_FILTER_SHIFT                        (11U)
#define CSL_ICSSECAP_ECCTL2_FILTER_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_ECCTL2_FILTER_MAX                          (0x0000001fU)

#define CSL_ICSSECAP_ECCTL2_RESETVAL                            (0x00000006U)

/* ECEINT */

#define CSL_ICSSECAP_ECEINT__RESV0_MASK                         (0x00000001U)
#define CSL_ICSSECAP_ECEINT__RESV0_SHIFT                        (0U)
#define CSL_ICSSECAP_ECEINT__RESV0_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_ECEINT__RESV0_MAX                          (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CEVT1_MASK                          (0x00000002U)
#define CSL_ICSSECAP_ECEINT_CEVT1_SHIFT                         (1U)
#define CSL_ICSSECAP_ECEINT_CEVT1_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CEVT1_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CEVT2_MASK                          (0x00000004U)
#define CSL_ICSSECAP_ECEINT_CEVT2_SHIFT                         (2U)
#define CSL_ICSSECAP_ECEINT_CEVT2_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CEVT2_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CEVT3_MASK                          (0x00000008U)
#define CSL_ICSSECAP_ECEINT_CEVT3_SHIFT                         (3U)
#define CSL_ICSSECAP_ECEINT_CEVT3_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CEVT3_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CEVT4_MASK                          (0x00000010U)
#define CSL_ICSSECAP_ECEINT_CEVT4_SHIFT                         (4U)
#define CSL_ICSSECAP_ECEINT_CEVT4_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CEVT4_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CNTOVF_MASK                         (0x00000020U)
#define CSL_ICSSECAP_ECEINT_CNTOVF_SHIFT                        (5U)
#define CSL_ICSSECAP_ECEINT_CNTOVF_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CNTOVF_MAX                          (0x00000001U)

#define CSL_ICSSECAP_ECEINT_PRDEQ_MASK                          (0x00000040U)
#define CSL_ICSSECAP_ECEINT_PRDEQ_SHIFT                         (6U)
#define CSL_ICSSECAP_ECEINT_PRDEQ_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_PRDEQ_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT_CMPEQ_MASK                          (0x00000080U)
#define CSL_ICSSECAP_ECEINT_CMPEQ_SHIFT                         (7U)
#define CSL_ICSSECAP_ECEINT_CMPEQ_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECEINT_CMPEQ_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECEINT__RESV1_MASK                         (0x0000FF00U)
#define CSL_ICSSECAP_ECEINT__RESV1_SHIFT                        (8U)
#define CSL_ICSSECAP_ECEINT__RESV1_RESETVAL                     (0x00000000U)
#define CSL_ICSSECAP_ECEINT__RESV1_MAX                          (0x000000ffU)

#define CSL_ICSSECAP_ECEINT_RESETVAL                            (0x00000000U)

/* ECFLG */

#define CSL_ICSSECAP_ECFLG_INT_MASK                             (0x00000001U)
#define CSL_ICSSECAP_ECFLG_INT_SHIFT                            (0U)
#define CSL_ICSSECAP_ECFLG_INT_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_ECFLG_INT_MAX                              (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CEVT1_MASK                           (0x00000002U)
#define CSL_ICSSECAP_ECFLG_CEVT1_SHIFT                          (1U)
#define CSL_ICSSECAP_ECFLG_CEVT1_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CEVT1_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CEVT2_MASK                           (0x00000004U)
#define CSL_ICSSECAP_ECFLG_CEVT2_SHIFT                          (2U)
#define CSL_ICSSECAP_ECFLG_CEVT2_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CEVT2_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CEVT3_MASK                           (0x00000008U)
#define CSL_ICSSECAP_ECFLG_CEVT3_SHIFT                          (3U)
#define CSL_ICSSECAP_ECFLG_CEVT3_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CEVT3_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CEVT4_MASK                           (0x00000010U)
#define CSL_ICSSECAP_ECFLG_CEVT4_SHIFT                          (4U)
#define CSL_ICSSECAP_ECFLG_CEVT4_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CEVT4_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CNTOVF_MASK                          (0x00000020U)
#define CSL_ICSSECAP_ECFLG_CNTOVF_SHIFT                         (5U)
#define CSL_ICSSECAP_ECFLG_CNTOVF_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CNTOVF_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECFLG_PRDEQ_MASK                           (0x00000040U)
#define CSL_ICSSECAP_ECFLG_PRDEQ_SHIFT                          (6U)
#define CSL_ICSSECAP_ECFLG_PRDEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_PRDEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG_CMPEQ_MASK                           (0x00000080U)
#define CSL_ICSSECAP_ECFLG_CMPEQ_SHIFT                          (7U)
#define CSL_ICSSECAP_ECFLG_CMPEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFLG_CMPEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFLG__RESV0_MASK                          (0x0000FF00U)
#define CSL_ICSSECAP_ECFLG__RESV0_SHIFT                         (8U)
#define CSL_ICSSECAP_ECFLG__RESV0_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECFLG__RESV0_MAX                           (0x000000ffU)

#define CSL_ICSSECAP_ECFLG_RESETVAL                             (0x00000000U)

/* ECCLR */

#define CSL_ICSSECAP_ECCLR_INT_MASK                             (0x00000001U)
#define CSL_ICSSECAP_ECCLR_INT_SHIFT                            (0U)
#define CSL_ICSSECAP_ECCLR_INT_RESETVAL                         (0x00000000U)
#define CSL_ICSSECAP_ECCLR_INT_MAX                              (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CEVT1_MASK                           (0x00000002U)
#define CSL_ICSSECAP_ECCLR_CEVT1_SHIFT                          (1U)
#define CSL_ICSSECAP_ECCLR_CEVT1_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CEVT1_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CEVT2_MASK                           (0x00000004U)
#define CSL_ICSSECAP_ECCLR_CEVT2_SHIFT                          (2U)
#define CSL_ICSSECAP_ECCLR_CEVT2_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CEVT2_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CEVT3_MASK                           (0x00000008U)
#define CSL_ICSSECAP_ECCLR_CEVT3_SHIFT                          (3U)
#define CSL_ICSSECAP_ECCLR_CEVT3_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CEVT3_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CEVT4_MASK                           (0x00000010U)
#define CSL_ICSSECAP_ECCLR_CEVT4_SHIFT                          (4U)
#define CSL_ICSSECAP_ECCLR_CEVT4_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CEVT4_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CNTOVF_MASK                          (0x00000020U)
#define CSL_ICSSECAP_ECCLR_CNTOVF_SHIFT                         (5U)
#define CSL_ICSSECAP_ECCLR_CNTOVF_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CNTOVF_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECCLR_PRDEQ_MASK                           (0x00000040U)
#define CSL_ICSSECAP_ECCLR_PRDEQ_SHIFT                          (6U)
#define CSL_ICSSECAP_ECCLR_PRDEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_PRDEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR_CMPEQ_MASK                           (0x00000080U)
#define CSL_ICSSECAP_ECCLR_CMPEQ_SHIFT                          (7U)
#define CSL_ICSSECAP_ECCLR_CMPEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECCLR_CMPEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECCLR__RESV0_MASK                          (0x0000FF00U)
#define CSL_ICSSECAP_ECCLR__RESV0_SHIFT                         (8U)
#define CSL_ICSSECAP_ECCLR__RESV0_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECCLR__RESV0_MAX                           (0x000000ffU)

#define CSL_ICSSECAP_ECCLR_RESETVAL                             (0x00000000U)

/* ECFRC */

#define CSL_ICSSECAP_ECFRC__RESV0_MASK                          (0x00000001U)
#define CSL_ICSSECAP_ECFRC__RESV0_SHIFT                         (0U)
#define CSL_ICSSECAP_ECFRC__RESV0_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECFRC__RESV0_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CEVT1_MASK                           (0x00000002U)
#define CSL_ICSSECAP_ECFRC_CEVT1_SHIFT                          (1U)
#define CSL_ICSSECAP_ECFRC_CEVT1_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CEVT1_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CEVT2_MASK                           (0x00000004U)
#define CSL_ICSSECAP_ECFRC_CEVT2_SHIFT                          (2U)
#define CSL_ICSSECAP_ECFRC_CEVT2_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CEVT2_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CEVT3_MASK                           (0x00000008U)
#define CSL_ICSSECAP_ECFRC_CEVT3_SHIFT                          (3U)
#define CSL_ICSSECAP_ECFRC_CEVT3_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CEVT3_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CEVT4_MASK                           (0x00000010U)
#define CSL_ICSSECAP_ECFRC_CEVT4_SHIFT                          (4U)
#define CSL_ICSSECAP_ECFRC_CEVT4_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CEVT4_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CNTOVF_MASK                          (0x00000020U)
#define CSL_ICSSECAP_ECFRC_CNTOVF_SHIFT                         (5U)
#define CSL_ICSSECAP_ECFRC_CNTOVF_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CNTOVF_MAX                           (0x00000001U)

#define CSL_ICSSECAP_ECFRC_PRDEQ_MASK                           (0x00000040U)
#define CSL_ICSSECAP_ECFRC_PRDEQ_SHIFT                          (6U)
#define CSL_ICSSECAP_ECFRC_PRDEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_PRDEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC_CMPEQ_MASK                           (0x00000080U)
#define CSL_ICSSECAP_ECFRC_CMPEQ_SHIFT                          (7U)
#define CSL_ICSSECAP_ECFRC_CMPEQ_RESETVAL                       (0x00000000U)
#define CSL_ICSSECAP_ECFRC_CMPEQ_MAX                            (0x00000001U)

#define CSL_ICSSECAP_ECFRC__RESV1_MASK                          (0x0000FF00U)
#define CSL_ICSSECAP_ECFRC__RESV1_SHIFT                         (8U)
#define CSL_ICSSECAP_ECFRC__RESV1_RESETVAL                      (0x00000000U)
#define CSL_ICSSECAP_ECFRC__RESV1_MAX                           (0x000000ffU)

#define CSL_ICSSECAP_ECFRC_RESETVAL                             (0x00000000U)

/* PID */

#define CSL_ICSSECAP_PID_REVID_MASK                             (0xFFFFFFFFU)
#define CSL_ICSSECAP_PID_REVID_SHIFT                            (0U)
#define CSL_ICSSECAP_PID_REVID_RESETVAL                         (0x44d22100U)
#define CSL_ICSSECAP_PID_REVID_MAX                              (0xffffffffU)

#define CSL_ICSSECAP_PID_RESETVAL                               (0x44d22100U)

#ifdef __cplusplus
}
#endif
#endif
