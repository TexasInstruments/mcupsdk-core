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

#ifndef CSLR_EQEP_H_
#define CSLR_EQEP_H_

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
    volatile uint32_t QPOSCNT;
    volatile uint32_t QPOSINIT;
    volatile uint32_t QPOSMAX;
    volatile uint32_t QPOSCMP;
    volatile uint32_t QPOSILAT;
    volatile uint32_t QPOSSLAT;
    volatile uint32_t QPOSLAT;
    volatile uint32_t QUTMR;
    volatile uint32_t QUPRD;
    volatile uint16_t QWDTMR;
    volatile uint16_t QWDPRD;
    volatile uint16_t QDECCTL;
    volatile uint16_t QEPCTL;
    volatile uint16_t QCAPCTL;
    volatile uint16_t QPOSCTL;
    volatile uint16_t QEINT;
    volatile uint16_t QFLG;
    volatile uint16_t QCLR;
    volatile uint16_t QFRC;
    volatile uint16_t QEPSTS;
    volatile uint16_t QCTMR;
    volatile uint16_t QCPRD;
    volatile uint16_t QCTMRLAT;
    volatile uint16_t QCPRDLAT;
    volatile uint8_t  Resv_96[30];
    volatile uint32_t REV;
    volatile uint32_t QEPSTROBESEL;
    volatile uint32_t QMACTRL;
    volatile uint32_t QEPSRCSEL;
} CSL_eqepRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_EQEP_QPOSCNT                                                       (0x00000000U)
#define CSL_EQEP_QPOSINIT                                                      (0x00000004U)
#define CSL_EQEP_QPOSMAX                                                       (0x00000008U)
#define CSL_EQEP_QPOSCMP                                                       (0x0000000CU)
#define CSL_EQEP_QPOSILAT                                                      (0x00000010U)
#define CSL_EQEP_QPOSSLAT                                                      (0x00000014U)
#define CSL_EQEP_QPOSLAT                                                       (0x00000018U)
#define CSL_EQEP_QUTMR                                                         (0x0000001CU)
#define CSL_EQEP_QUPRD                                                         (0x00000020U)
#define CSL_EQEP_QWDTMR                                                        (0x00000024U)
#define CSL_EQEP_QWDPRD                                                        (0x00000026U)
#define CSL_EQEP_QDECCTL                                                       (0x00000028U)
#define CSL_EQEP_QEPCTL                                                        (0x0000002AU)
#define CSL_EQEP_QCAPCTL                                                       (0x0000002CU)
#define CSL_EQEP_QPOSCTL                                                       (0x0000002EU)
#define CSL_EQEP_QEINT                                                         (0x00000030U)
#define CSL_EQEP_QFLG                                                          (0x00000032U)
#define CSL_EQEP_QCLR                                                          (0x00000034U)
#define CSL_EQEP_QFRC                                                          (0x00000036U)
#define CSL_EQEP_QEPSTS                                                        (0x00000038U)
#define CSL_EQEP_QCTMR                                                         (0x0000003AU)
#define CSL_EQEP_QCPRD                                                         (0x0000003CU)
#define CSL_EQEP_QCTMRLAT                                                      (0x0000003EU)
#define CSL_EQEP_QCPRDLAT                                                      (0x00000040U)
#define CSL_EQEP_REV                                                           (0x00000060U)
#define CSL_EQEP_QEPSTROBESEL                                                  (0x00000064U)
#define CSL_EQEP_QMACTRL                                                       (0x00000068U)
#define CSL_EQEP_QEPSRCSEL                                                     (0x0000006CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* QPOSCNT */

#define CSL_EQEP_QPOSCNT_QPOSCNT_MASK                                          (0xFFFFFFFFU)
#define CSL_EQEP_QPOSCNT_QPOSCNT_SHIFT                                         (0x00000000U)
#define CSL_EQEP_QPOSCNT_QPOSCNT_RESETVAL                                      (0x00000000U)
#define CSL_EQEP_QPOSCNT_QPOSCNT_MAX                                           (0xFFFFFFFFU)

#define CSL_EQEP_QPOSCNT_RESETVAL                                              (0x00000000U)

/* QPOSINIT */

#define CSL_EQEP_QPOSINIT_QPOSINIT_MASK                                        (0xFFFFFFFFU)
#define CSL_EQEP_QPOSINIT_QPOSINIT_SHIFT                                       (0x00000000U)
#define CSL_EQEP_QPOSINIT_QPOSINIT_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QPOSINIT_QPOSINIT_MAX                                         (0xFFFFFFFFU)

#define CSL_EQEP_QPOSINIT_RESETVAL                                             (0x00000000U)

/* QPOSMAX */

#define CSL_EQEP_QPOSMAX_QPOSMAX_MASK                                          (0xFFFFFFFFU)
#define CSL_EQEP_QPOSMAX_QPOSMAX_SHIFT                                         (0x00000000U)
#define CSL_EQEP_QPOSMAX_QPOSMAX_RESETVAL                                      (0x00000000U)
#define CSL_EQEP_QPOSMAX_QPOSMAX_MAX                                           (0xFFFFFFFFU)

#define CSL_EQEP_QPOSMAX_RESETVAL                                              (0x00000000U)

/* QPOSCMP */

#define CSL_EQEP_QPOSCMP_QPOSCMP_MASK                                          (0xFFFFFFFFU)
#define CSL_EQEP_QPOSCMP_QPOSCMP_SHIFT                                         (0x00000000U)
#define CSL_EQEP_QPOSCMP_QPOSCMP_RESETVAL                                      (0x00000000U)
#define CSL_EQEP_QPOSCMP_QPOSCMP_MAX                                           (0xFFFFFFFFU)

#define CSL_EQEP_QPOSCMP_RESETVAL                                              (0x00000000U)

/* QPOSILAT */

#define CSL_EQEP_QPOSILAT_QPOSILAT_MASK                                        (0xFFFFFFFFU)
#define CSL_EQEP_QPOSILAT_QPOSILAT_SHIFT                                       (0x00000000U)
#define CSL_EQEP_QPOSILAT_QPOSILAT_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QPOSILAT_QPOSILAT_MAX                                         (0xFFFFFFFFU)

#define CSL_EQEP_QPOSILAT_RESETVAL                                             (0x00000000U)

/* QPOSSLAT */

#define CSL_EQEP_QPOSSLAT_QPOSSLAT_MASK                                        (0xFFFFFFFFU)
#define CSL_EQEP_QPOSSLAT_QPOSSLAT_SHIFT                                       (0x00000000U)
#define CSL_EQEP_QPOSSLAT_QPOSSLAT_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QPOSSLAT_QPOSSLAT_MAX                                         (0xFFFFFFFFU)

#define CSL_EQEP_QPOSSLAT_RESETVAL                                             (0x00000000U)

/* QPOSLAT */

#define CSL_EQEP_QPOSLAT_QPOSLAT_MASK                                          (0xFFFFFFFFU)
#define CSL_EQEP_QPOSLAT_QPOSLAT_SHIFT                                         (0x00000000U)
#define CSL_EQEP_QPOSLAT_QPOSLAT_RESETVAL                                      (0x00000000U)
#define CSL_EQEP_QPOSLAT_QPOSLAT_MAX                                           (0xFFFFFFFFU)

#define CSL_EQEP_QPOSLAT_RESETVAL                                              (0x00000000U)

/* QUTMR */

#define CSL_EQEP_QUTMR_QUTMR_MASK                                              (0xFFFFFFFFU)
#define CSL_EQEP_QUTMR_QUTMR_SHIFT                                             (0x00000000U)
#define CSL_EQEP_QUTMR_QUTMR_RESETVAL                                          (0x00000000U)
#define CSL_EQEP_QUTMR_QUTMR_MAX                                               (0xFFFFFFFFU)

#define CSL_EQEP_QUTMR_RESETVAL                                                (0x00000000U)

/* QUPRD */

#define CSL_EQEP_QUPRD_QUPRD_MASK                                              (0xFFFFFFFFU)
#define CSL_EQEP_QUPRD_QUPRD_SHIFT                                             (0x00000000U)
#define CSL_EQEP_QUPRD_QUPRD_RESETVAL                                          (0x00000000U)
#define CSL_EQEP_QUPRD_QUPRD_MAX                                               (0xFFFFFFFFU)

#define CSL_EQEP_QUPRD_RESETVAL                                                (0x00000000U)

/* QWDTMR */

#define CSL_EQEP_QWDTMR_QWDTMR_MASK                                            (0xFFFFU)
#define CSL_EQEP_QWDTMR_QWDTMR_SHIFT                                           (0x0000U)
#define CSL_EQEP_QWDTMR_QWDTMR_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QWDTMR_QWDTMR_MAX                                             (0xFFFFU)

#define CSL_EQEP_QWDTMR_RESETVAL                                               (0x0000U)

/* QWDPRD */

#define CSL_EQEP_QWDPRD_QWDPRD_MASK                                            (0xFFFFU)
#define CSL_EQEP_QWDPRD_QWDPRD_SHIFT                                           (0x0000U)
#define CSL_EQEP_QWDPRD_QWDPRD_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QWDPRD_QWDPRD_MAX                                             (0xFFFFU)

#define CSL_EQEP_QWDPRD_RESETVAL                                               (0x0000U)

/* QDECCTL */

#define CSL_EQEP_QDECCTL_QIDIRE_MASK                                           (0x0001U)
#define CSL_EQEP_QDECCTL_QIDIRE_SHIFT                                          (0x0000U)
#define CSL_EQEP_QDECCTL_QIDIRE_RESETVAL                                       (0x0000U)
#define CSL_EQEP_QDECCTL_QIDIRE_MAX                                            (0x0001U)

#define CSL_EQEP_QDECCTL_RESERVED_1_MASK                                       (0x001EU)
#define CSL_EQEP_QDECCTL_RESERVED_1_SHIFT                                      (0x0001U)
#define CSL_EQEP_QDECCTL_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_EQEP_QDECCTL_RESERVED_1_MAX                                        (0x000FU)

#define CSL_EQEP_QDECCTL_QSP_MASK                                              (0x0020U)
#define CSL_EQEP_QDECCTL_QSP_SHIFT                                             (0x0005U)
#define CSL_EQEP_QDECCTL_QSP_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QDECCTL_QSP_MAX                                               (0x0001U)

#define CSL_EQEP_QDECCTL_QSP_VAL_QSP_NOPOLAR                                   (0x0U)
#define CSL_EQEP_QDECCTL_QSP_VAL_QSP_POLAR                                     (0x1U)

#define CSL_EQEP_QDECCTL_QIP_MASK                                              (0x0040U)
#define CSL_EQEP_QDECCTL_QIP_SHIFT                                             (0x0006U)
#define CSL_EQEP_QDECCTL_QIP_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QDECCTL_QIP_MAX                                               (0x0001U)

#define CSL_EQEP_QDECCTL_QIP_VAL_QIP_NOPOLAR                                   (0x0U)
#define CSL_EQEP_QDECCTL_QIP_VAL_QIP_POLAR                                     (0x1U)

#define CSL_EQEP_QDECCTL_QBP_MASK                                              (0x0080U)
#define CSL_EQEP_QDECCTL_QBP_SHIFT                                             (0x0007U)
#define CSL_EQEP_QDECCTL_QBP_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QDECCTL_QBP_MAX                                               (0x0001U)

#define CSL_EQEP_QDECCTL_QBP_VAL_QBP_NOPOLAR                                   (0x0U)
#define CSL_EQEP_QDECCTL_QBP_VAL_QBP_POLAR                                     (0x1U)

#define CSL_EQEP_QDECCTL_QAP_MASK                                              (0x0100U)
#define CSL_EQEP_QDECCTL_QAP_SHIFT                                             (0x0008U)
#define CSL_EQEP_QDECCTL_QAP_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QDECCTL_QAP_MAX                                               (0x0001U)

#define CSL_EQEP_QDECCTL_QAP_VAL_QAP_NOPOLAR                                   (0x0U)
#define CSL_EQEP_QDECCTL_QAP_VAL_QAP_POLAR                                     (0x1U)

#define CSL_EQEP_QDECCTL_IGATE_MASK                                            (0x0200U)
#define CSL_EQEP_QDECCTL_IGATE_SHIFT                                           (0x0009U)
#define CSL_EQEP_QDECCTL_IGATE_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QDECCTL_IGATE_MAX                                             (0x0001U)

#define CSL_EQEP_QDECCTL_IGATE_VAL_IGATE_DISABLE                               (0x0U)
#define CSL_EQEP_QDECCTL_IGATE_VAL_IGATE_ENABLE                                (0x1U)

#define CSL_EQEP_QDECCTL_SWAP_MASK                                             (0x0400U)
#define CSL_EQEP_QDECCTL_SWAP_SHIFT                                            (0x000AU)
#define CSL_EQEP_QDECCTL_SWAP_RESETVAL                                         (0x0000U)
#define CSL_EQEP_QDECCTL_SWAP_MAX                                              (0x0001U)

#define CSL_EQEP_QDECCTL_SWAP_VAL_SWAP_DISABLE                                 (0x0U)
#define CSL_EQEP_QDECCTL_SWAP_VAL_SWAP_ENABLE                                  (0x1U)

#define CSL_EQEP_QDECCTL_XCR_MASK                                              (0x0800U)
#define CSL_EQEP_QDECCTL_XCR_SHIFT                                             (0x000BU)
#define CSL_EQEP_QDECCTL_XCR_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QDECCTL_XCR_MAX                                               (0x0001U)

#define CSL_EQEP_QDECCTL_XCR_VAL_XCR_2XRESOL                                   (0x0U)
#define CSL_EQEP_QDECCTL_XCR_VAL_XCR_1XRESOL                                   (0x1U)

#define CSL_EQEP_QDECCTL_SPSEL_MASK                                            (0x1000U)
#define CSL_EQEP_QDECCTL_SPSEL_SHIFT                                           (0x000CU)
#define CSL_EQEP_QDECCTL_SPSEL_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QDECCTL_SPSEL_MAX                                             (0x0001U)

#define CSL_EQEP_QDECCTL_SPSEL_VAL_INDEX_PIN                                   (0x0U)
#define CSL_EQEP_QDECCTL_SPSEL_VAL_STROBE_PIN                                  (0x1U)

#define CSL_EQEP_QDECCTL_SOEN_MASK                                             (0x2000U)
#define CSL_EQEP_QDECCTL_SOEN_SHIFT                                            (0x000DU)
#define CSL_EQEP_QDECCTL_SOEN_RESETVAL                                         (0x0000U)
#define CSL_EQEP_QDECCTL_SOEN_MAX                                              (0x0001U)

#define CSL_EQEP_QDECCTL_SOEN_VAL_SYNC_DISABLE                                 (0x0U)
#define CSL_EQEP_QDECCTL_SOEN_VAL_SYNC_ENABLE                                  (0x1U)

#define CSL_EQEP_QDECCTL_QSRC_MASK                                             (0xC000U)
#define CSL_EQEP_QDECCTL_QSRC_SHIFT                                            (0x000EU)
#define CSL_EQEP_QDECCTL_QSRC_RESETVAL                                         (0x0000U)
#define CSL_EQEP_QDECCTL_QSRC_MAX                                              (0x0003U)

#define CSL_EQEP_QDECCTL_RESETVAL                                              (0x0000U)

/* QEPCTL */

#define CSL_EQEP_QEPCTL_WDE_MASK                                               (0x0001U)
#define CSL_EQEP_QEPCTL_WDE_SHIFT                                              (0x0000U)
#define CSL_EQEP_QEPCTL_WDE_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_WDE_MAX                                                (0x0001U)

#define CSL_EQEP_QEPCTL_WDE_VAL_WDE_DISABLE                                    (0x0U)
#define CSL_EQEP_QEPCTL_WDE_VAL_WDE_ENABLE                                     (0x1U)

#define CSL_EQEP_QEPCTL_UTE_MASK                                               (0x0002U)
#define CSL_EQEP_QEPCTL_UTE_SHIFT                                              (0x0001U)
#define CSL_EQEP_QEPCTL_UTE_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_UTE_MAX                                                (0x0001U)

#define CSL_EQEP_QEPCTL_UTE_VAL_UTE_DISABLE                                    (0x0U)
#define CSL_EQEP_QEPCTL_UTE_VAL_UTE_ENABLE                                     (0x1U)

#define CSL_EQEP_QEPCTL_QCLM_MASK                                              (0x0004U)
#define CSL_EQEP_QEPCTL_QCLM_SHIFT                                             (0x0002U)
#define CSL_EQEP_QEPCTL_QCLM_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPCTL_QCLM_MAX                                               (0x0001U)

#define CSL_EQEP_QEPCTL_QCLM_VAL_QCLM_CPU                                      (0x0U)
#define CSL_EQEP_QEPCTL_QCLM_VAL_QCLM_TIMEOUT                                  (0x1U)

#define CSL_EQEP_QEPCTL_QPEN_MASK                                              (0x0008U)
#define CSL_EQEP_QEPCTL_QPEN_SHIFT                                             (0x0003U)
#define CSL_EQEP_QEPCTL_QPEN_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPCTL_QPEN_MAX                                               (0x0001U)

#define CSL_EQEP_QEPCTL_QPEN_VAL_QPEN_RESET                                    (0x0U)
#define CSL_EQEP_QEPCTL_QPEN_VAL_QPEN_ENABLE                                   (0x1U)

#define CSL_EQEP_QEPCTL_IEL_MASK                                               (0x0030U)
#define CSL_EQEP_QEPCTL_IEL_SHIFT                                              (0x0004U)
#define CSL_EQEP_QEPCTL_IEL_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_IEL_MAX                                                (0x0003U)

#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_RSVD                                       (0x0U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_POSRISING                                  (0x1U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_POSFALLING                                 (0x2U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_SIM                                        (0x3U)

#define CSL_EQEP_QEPCTL_SEL_MASK                                               (0x0040U)
#define CSL_EQEP_QEPCTL_SEL_SHIFT                                              (0x0006U)
#define CSL_EQEP_QEPCTL_SEL_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_SEL_MAX                                                (0x0001U)

#define CSL_EQEP_QEPCTL_SEL_VAL_SEL_QEPSRISING                                 (0x0U)
#define CSL_EQEP_QEPCTL_SEL_VAL_SEL_QEPSCLOCK                                  (0x1U)

#define CSL_EQEP_QEPCTL_SWI_MASK                                               (0x0080U)
#define CSL_EQEP_QEPCTL_SWI_SHIFT                                              (0x0007U)
#define CSL_EQEP_QEPCTL_SWI_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_SWI_MAX                                                (0x0001U)

#define CSL_EQEP_QEPCTL_SWI_VAL_SWI_NOTHING                                    (0x0U)
#define CSL_EQEP_QEPCTL_SWI_VAL_SWI_INITPOS                                    (0x1U)

#define CSL_EQEP_QEPCTL_IEI_MASK                                               (0x0300U)
#define CSL_EQEP_QEPCTL_IEI_SHIFT                                              (0x0008U)
#define CSL_EQEP_QEPCTL_IEI_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_IEI_MAX                                                (0x0003U)

#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_NOTHING0                                   (0x0U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_NOTHING1                                   (0x1U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_INITRISING                                 (0x2U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_INITFALLING                                (0x3U)

#define CSL_EQEP_QEPCTL_SEI_MASK                                               (0x0C00U)
#define CSL_EQEP_QEPCTL_SEI_SHIFT                                              (0x000AU)
#define CSL_EQEP_QEPCTL_SEI_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPCTL_SEI_MAX                                                (0x0003U)

#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_NOTHING0                                   (0x0U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_NOTHING1                                   (0x1U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_INITQEPSRISING                             (0x2U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_INITQEPSCLOCK                              (0x3U)

#define CSL_EQEP_QEPCTL_PCRM_MASK                                              (0x3000U)
#define CSL_EQEP_QEPCTL_PCRM_SHIFT                                             (0x000CU)
#define CSL_EQEP_QEPCTL_PCRM_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPCTL_PCRM_MAX                                               (0x0003U)

#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_INDEX                                    (0x0U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_MAXPOS                                   (0x1U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_FIRSTINDEX                               (0x2U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_TIMEEVENT                                (0x3U)

#define CSL_EQEP_QEPCTL_FREE_SOFT_MASK                                         (0xC000U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_SHIFT                                        (0x000EU)
#define CSL_EQEP_QEPCTL_FREE_SOFT_RESETVAL                                     (0x0000U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_MAX                                          (0x0003U)

#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_0                              (0x0U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_1                              (0x1U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_2                              (0x2U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_3                              (0x3U)

#define CSL_EQEP_QEPCTL_RESETVAL                                               (0x0000U)

/* QCAPCTL */

#define CSL_EQEP_QCAPCTL_UPPS_MASK                                             (0x000FU)
#define CSL_EQEP_QCAPCTL_UPPS_SHIFT                                            (0x0000U)
#define CSL_EQEP_QCAPCTL_UPPS_RESETVAL                                         (0x0000U)
#define CSL_EQEP_QCAPCTL_UPPS_MAX                                              (0x000FU)

#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK1                                        (0x0U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK2                                        (0x1U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK4                                        (0x2U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK8                                        (0x3U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK16                                       (0x4U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK32                                       (0x5U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK64                                       (0x6U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK128                                      (0x7U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK256                                      (0x8U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK512                                      (0x9U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK1024                                     (0xAU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK2048                                     (0xBU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD0                                   (0xCU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD1                                   (0xDU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD2                                   (0xEU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD3                                   (0xFU)

#define CSL_EQEP_QCAPCTL_CCPS_MASK                                             (0x0070U)
#define CSL_EQEP_QCAPCTL_CCPS_SHIFT                                            (0x0004U)
#define CSL_EQEP_QCAPCTL_CCPS_RESETVAL                                         (0x0000U)
#define CSL_EQEP_QCAPCTL_CCPS_MAX                                              (0x0007U)

#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT1                                   (0x0U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT2                                   (0x1U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT4                                   (0x2U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT8                                   (0x3U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT16                                  (0x4U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT32                                  (0x5U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT64                                  (0x6U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT128                                 (0x7U)

#define CSL_EQEP_QCAPCTL_RESERVED_1_MASK                                       (0x7F80U)
#define CSL_EQEP_QCAPCTL_RESERVED_1_SHIFT                                      (0x0007U)
#define CSL_EQEP_QCAPCTL_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_EQEP_QCAPCTL_RESERVED_1_MAX                                        (0x00FFU)

#define CSL_EQEP_QCAPCTL_CEN_MASK                                              (0x8000U)
#define CSL_EQEP_QCAPCTL_CEN_SHIFT                                             (0x000FU)
#define CSL_EQEP_QCAPCTL_CEN_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QCAPCTL_CEN_MAX                                               (0x0001U)

#define CSL_EQEP_QCAPCTL_CEN_VAL_CEN_DISABLE                                   (0x0U)
#define CSL_EQEP_QCAPCTL_CEN_VAL_CEN_ENABLE                                    (0x1U)

#define CSL_EQEP_QCAPCTL_RESETVAL                                              (0x0000U)

/* QPOSCTL */

#define CSL_EQEP_QPOSCTL_PCSPW_MASK                                            (0x0FFFU)
#define CSL_EQEP_QPOSCTL_PCSPW_SHIFT                                           (0x0000U)
#define CSL_EQEP_QPOSCTL_PCSPW_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QPOSCTL_PCSPW_MAX                                             (0x0FFFU)

#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT4                                  (0x0U)
#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT8                                  (0x1U)
#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT16384                              (0xFFFU)

#define CSL_EQEP_QPOSCTL_PCE_MASK                                              (0x1000U)
#define CSL_EQEP_QPOSCTL_PCE_SHIFT                                             (0x000CU)
#define CSL_EQEP_QPOSCTL_PCE_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QPOSCTL_PCE_MAX                                               (0x0001U)

#define CSL_EQEP_QPOSCTL_PCE_VAL_PCE_DISABLE                                   (0x0U)
#define CSL_EQEP_QPOSCTL_PCE_VAL_PCE_ENABLE                                    (0x1U)

#define CSL_EQEP_QPOSCTL_PCPOL_MASK                                            (0x2000U)
#define CSL_EQEP_QPOSCTL_PCPOL_SHIFT                                           (0x000DU)
#define CSL_EQEP_QPOSCTL_PCPOL_RESETVAL                                        (0x0000U)
#define CSL_EQEP_QPOSCTL_PCPOL_MAX                                             (0x0001U)

#define CSL_EQEP_QPOSCTL_PCPOL_VAL_PCPOL_HIGH                                  (0x0U)
#define CSL_EQEP_QPOSCTL_PCPOL_VAL_PCPOL_LOW                                   (0x1U)

#define CSL_EQEP_QPOSCTL_PCLOAD_MASK                                           (0x4000U)
#define CSL_EQEP_QPOSCTL_PCLOAD_SHIFT                                          (0x000EU)
#define CSL_EQEP_QPOSCTL_PCLOAD_RESETVAL                                       (0x0000U)
#define CSL_EQEP_QPOSCTL_PCLOAD_MAX                                            (0x0001U)

#define CSL_EQEP_QPOSCTL_PCLOAD_VAL_PCLOAD_0                                   (0x0U)
#define CSL_EQEP_QPOSCTL_PCLOAD_VAL_PCLOAD_QPOSCMP                             (0x1U)

#define CSL_EQEP_QPOSCTL_PCSHDW_MASK                                           (0x8000U)
#define CSL_EQEP_QPOSCTL_PCSHDW_SHIFT                                          (0x000FU)
#define CSL_EQEP_QPOSCTL_PCSHDW_RESETVAL                                       (0x0000U)
#define CSL_EQEP_QPOSCTL_PCSHDW_MAX                                            (0x0001U)

#define CSL_EQEP_QPOSCTL_PCSHDW_VAL_PCSHDW_DISABLE                             (0x0U)
#define CSL_EQEP_QPOSCTL_PCSHDW_VAL_PCSHDW_ENABLE                              (0x1U)

#define CSL_EQEP_QPOSCTL_RESETVAL                                              (0x0000U)

/* QEINT */

#define CSL_EQEP_QEINT_RESERVED_1_MASK                                         (0x0001U)
#define CSL_EQEP_QEINT_RESERVED_1_SHIFT                                        (0x0000U)
#define CSL_EQEP_QEINT_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_EQEP_QEINT_RESERVED_1_MAX                                          (0x0001U)

#define CSL_EQEP_QEINT_PCE_MASK                                                (0x0002U)
#define CSL_EQEP_QEINT_PCE_SHIFT                                               (0x0001U)
#define CSL_EQEP_QEINT_PCE_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_PCE_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_PCE_VAL_PCE_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_PCE_VAL_PCE_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_QPE_MASK                                                (0x0004U)
#define CSL_EQEP_QEINT_QPE_SHIFT                                               (0x0002U)
#define CSL_EQEP_QEINT_QPE_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_QPE_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_QPE_VAL_QPE_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_QPE_VAL_QPE_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_QDC_MASK                                                (0x0008U)
#define CSL_EQEP_QEINT_QDC_SHIFT                                               (0x0003U)
#define CSL_EQEP_QEINT_QDC_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_QDC_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_QDC_VAL_QDC_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_QDC_VAL_QDC_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_WTO_MASK                                                (0x0010U)
#define CSL_EQEP_QEINT_WTO_SHIFT                                               (0x0004U)
#define CSL_EQEP_QEINT_WTO_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_WTO_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_WTO_VAL_WTO_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_WTO_VAL_WTO_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_PCU_MASK                                                (0x0020U)
#define CSL_EQEP_QEINT_PCU_SHIFT                                               (0x0005U)
#define CSL_EQEP_QEINT_PCU_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_PCU_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_PCU_VAL_PCU_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_PCU_VAL_PCU_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_PCO_MASK                                                (0x0040U)
#define CSL_EQEP_QEINT_PCO_SHIFT                                               (0x0006U)
#define CSL_EQEP_QEINT_PCO_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_PCO_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_PCO_VAL_PCO_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_PCO_VAL_PCO_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_PCR_MASK                                                (0x0080U)
#define CSL_EQEP_QEINT_PCR_SHIFT                                               (0x0007U)
#define CSL_EQEP_QEINT_PCR_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_PCR_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_PCR_VAL_PCR_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_PCR_VAL_PCR_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_PCM_MASK                                                (0x0100U)
#define CSL_EQEP_QEINT_PCM_SHIFT                                               (0x0008U)
#define CSL_EQEP_QEINT_PCM_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_PCM_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_PCM_VAL_PCM_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_PCM_VAL_PCM_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_SEL_MASK                                                (0x0200U)
#define CSL_EQEP_QEINT_SEL_SHIFT                                               (0x0009U)
#define CSL_EQEP_QEINT_SEL_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_SEL_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_SEL_VAL_SEL_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_SEL_VAL_SEL_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_IEL_MASK                                                (0x0400U)
#define CSL_EQEP_QEINT_IEL_SHIFT                                               (0x000AU)
#define CSL_EQEP_QEINT_IEL_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_IEL_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_IEL_VAL_IEL_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_IEL_VAL_IEL_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_UTO_MASK                                                (0x0800U)
#define CSL_EQEP_QEINT_UTO_SHIFT                                               (0x000BU)
#define CSL_EQEP_QEINT_UTO_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QEINT_UTO_MAX                                                 (0x0001U)

#define CSL_EQEP_QEINT_UTO_VAL_UTO_DISABLE                                     (0x0U)
#define CSL_EQEP_QEINT_UTO_VAL_UTO_ENABLE                                      (0x1U)

#define CSL_EQEP_QEINT_QMAE_MASK                                               (0x1000U)
#define CSL_EQEP_QEINT_QMAE_SHIFT                                              (0x000CU)
#define CSL_EQEP_QEINT_QMAE_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEINT_QMAE_MAX                                                (0x0001U)

#define CSL_EQEP_QEINT_QMAE_VAL_QMAE_DISABLE                                   (0x0U)
#define CSL_EQEP_QEINT_QMAE_VAL_QMAE_ENABLE                                    (0x1U)

#define CSL_EQEP_QEINT_RESERVED_2_MASK                                         (0xE000U)
#define CSL_EQEP_QEINT_RESERVED_2_SHIFT                                        (0x000DU)
#define CSL_EQEP_QEINT_RESERVED_2_RESETVAL                                     (0x0000U)
#define CSL_EQEP_QEINT_RESERVED_2_MAX                                          (0x0007U)

#define CSL_EQEP_QEINT_RESETVAL                                                (0x0000U)

/* QFLG */

#define CSL_EQEP_QFLG_INT_MASK                                                 (0x0001U)
#define CSL_EQEP_QFLG_INT_SHIFT                                                (0x0000U)
#define CSL_EQEP_QFLG_INT_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_INT_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_INT_VAL_INT_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_INT_VAL_INT_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PCE_MASK                                                 (0x0002U)
#define CSL_EQEP_QFLG_PCE_SHIFT                                                (0x0001U)
#define CSL_EQEP_QFLG_PCE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PCE_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PCE_VAL_PCE_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PCE_VAL_PCE_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PHE_MASK                                                 (0x0004U)
#define CSL_EQEP_QFLG_PHE_SHIFT                                                (0x0002U)
#define CSL_EQEP_QFLG_PHE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PHE_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PHE_VAL_PHE_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PHE_VAL_PHE_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_QDC_MASK                                                 (0x0008U)
#define CSL_EQEP_QFLG_QDC_SHIFT                                                (0x0003U)
#define CSL_EQEP_QFLG_QDC_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_QDC_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_QDC_VAL_QDC_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_QDC_VAL_QDC_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_WTO_MASK                                                 (0x0010U)
#define CSL_EQEP_QFLG_WTO_SHIFT                                                (0x0004U)
#define CSL_EQEP_QFLG_WTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_WTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_WTO_VAL_WTO_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_WTO_VAL_WTO_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PCU_MASK                                                 (0x0020U)
#define CSL_EQEP_QFLG_PCU_SHIFT                                                (0x0005U)
#define CSL_EQEP_QFLG_PCU_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PCU_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PCU_VAL_PCU_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PCU_VAL_PCU_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PCO_MASK                                                 (0x0040U)
#define CSL_EQEP_QFLG_PCO_SHIFT                                                (0x0006U)
#define CSL_EQEP_QFLG_PCO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PCO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PCO_VAL_PCO_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PCO_VAL_PCO_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PCR_MASK                                                 (0x0080U)
#define CSL_EQEP_QFLG_PCR_SHIFT                                                (0x0007U)
#define CSL_EQEP_QFLG_PCR_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PCR_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PCR_VAL_PCR_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PCR_VAL_PCR_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_PCM_MASK                                                 (0x0100U)
#define CSL_EQEP_QFLG_PCM_SHIFT                                                (0x0008U)
#define CSL_EQEP_QFLG_PCM_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_PCM_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_PCM_VAL_PCM_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_PCM_VAL_PCM_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_SEL_MASK                                                 (0x0200U)
#define CSL_EQEP_QFLG_SEL_SHIFT                                                (0x0009U)
#define CSL_EQEP_QFLG_SEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_SEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_SEL_VAL_SEL_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_SEL_VAL_SEL_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_IEL_MASK                                                 (0x0400U)
#define CSL_EQEP_QFLG_IEL_SHIFT                                                (0x000AU)
#define CSL_EQEP_QFLG_IEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_IEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_IEL_VAL_IEL_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_IEL_VAL_IEL_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_UTO_MASK                                                 (0x0800U)
#define CSL_EQEP_QFLG_UTO_SHIFT                                                (0x000BU)
#define CSL_EQEP_QFLG_UTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFLG_UTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFLG_UTO_VAL_UTO_NOFLAG                                       (0x0U)
#define CSL_EQEP_QFLG_UTO_VAL_UTO_FLAG                                         (0x1U)

#define CSL_EQEP_QFLG_QMAE_MASK                                                (0x1000U)
#define CSL_EQEP_QFLG_QMAE_SHIFT                                               (0x000CU)
#define CSL_EQEP_QFLG_QMAE_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QFLG_QMAE_MAX                                                 (0x0001U)

#define CSL_EQEP_QFLG_QMAE_VAL_QMAE_NOFLAG                                     (0x0U)
#define CSL_EQEP_QFLG_QMAE_VAL_QMAE_FLAG                                       (0x1U)

#define CSL_EQEP_QFLG_RESERVED_1_MASK                                          (0xE000U)
#define CSL_EQEP_QFLG_RESERVED_1_SHIFT                                         (0x000DU)
#define CSL_EQEP_QFLG_RESERVED_1_RESETVAL                                      (0x0000U)
#define CSL_EQEP_QFLG_RESERVED_1_MAX                                           (0x0007U)

#define CSL_EQEP_QFLG_RESETVAL                                                 (0x0000U)

/* QCLR */

#define CSL_EQEP_QCLR_INT_MASK                                                 (0x0001U)
#define CSL_EQEP_QCLR_INT_SHIFT                                                (0x0000U)
#define CSL_EQEP_QCLR_INT_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_INT_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_INT_VAL_INT_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_INT_VAL_INT_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PCE_MASK                                                 (0x0002U)
#define CSL_EQEP_QCLR_PCE_SHIFT                                                (0x0001U)
#define CSL_EQEP_QCLR_PCE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PCE_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PCE_VAL_PCE_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PCE_VAL_PCE_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PHE_MASK                                                 (0x0004U)
#define CSL_EQEP_QCLR_PHE_SHIFT                                                (0x0002U)
#define CSL_EQEP_QCLR_PHE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PHE_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PHE_VAL_PHE_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PHE_VAL_PHE_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_QDC_MASK                                                 (0x0008U)
#define CSL_EQEP_QCLR_QDC_SHIFT                                                (0x0003U)
#define CSL_EQEP_QCLR_QDC_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_QDC_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_QDC_VAL_QDC_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_QDC_VAL_QDC_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_WTO_MASK                                                 (0x0010U)
#define CSL_EQEP_QCLR_WTO_SHIFT                                                (0x0004U)
#define CSL_EQEP_QCLR_WTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_WTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_WTO_VAL_WTO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_WTO_VAL_WTO_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PCU_MASK                                                 (0x0020U)
#define CSL_EQEP_QCLR_PCU_SHIFT                                                (0x0005U)
#define CSL_EQEP_QCLR_PCU_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PCU_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PCU_VAL_PCU_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PCU_VAL_PCU_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PCO_MASK                                                 (0x0040U)
#define CSL_EQEP_QCLR_PCO_SHIFT                                                (0x0006U)
#define CSL_EQEP_QCLR_PCO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PCO_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PCO_VAL_PCO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PCO_VAL_PCO_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PCR_MASK                                                 (0x0080U)
#define CSL_EQEP_QCLR_PCR_SHIFT                                                (0x0007U)
#define CSL_EQEP_QCLR_PCR_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PCR_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PCR_VAL_PCR_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PCR_VAL_PCR_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_PCM_MASK                                                 (0x0100U)
#define CSL_EQEP_QCLR_PCM_SHIFT                                                (0x0008U)
#define CSL_EQEP_QCLR_PCM_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_PCM_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_PCM_VAL_PCM_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_PCM_VAL_PCM_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_SEL_MASK                                                 (0x0200U)
#define CSL_EQEP_QCLR_SEL_SHIFT                                                (0x0009U)
#define CSL_EQEP_QCLR_SEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_SEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_SEL_VAL_SEL_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_SEL_VAL_SEL_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_IEL_MASK                                                 (0x0400U)
#define CSL_EQEP_QCLR_IEL_SHIFT                                                (0x000AU)
#define CSL_EQEP_QCLR_IEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_IEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_IEL_VAL_IEL_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_IEL_VAL_IEL_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_UTO_MASK                                                 (0x0800U)
#define CSL_EQEP_QCLR_UTO_SHIFT                                                (0x000BU)
#define CSL_EQEP_QCLR_UTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QCLR_UTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QCLR_UTO_VAL_UTO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QCLR_UTO_VAL_UTO_CLR                                          (0x1U)

#define CSL_EQEP_QCLR_QMAE_MASK                                                (0x1000U)
#define CSL_EQEP_QCLR_QMAE_SHIFT                                               (0x000CU)
#define CSL_EQEP_QCLR_QMAE_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QCLR_QMAE_MAX                                                 (0x0001U)

#define CSL_EQEP_QCLR_QMAE_VAL_QMAE_NOEFFECT                                   (0x0U)
#define CSL_EQEP_QCLR_QMAE_VAL_QMAE_CLR                                        (0x1U)

#define CSL_EQEP_QCLR_RESERVED_1_MASK                                          (0xE000U)
#define CSL_EQEP_QCLR_RESERVED_1_SHIFT                                         (0x000DU)
#define CSL_EQEP_QCLR_RESERVED_1_RESETVAL                                      (0x0000U)
#define CSL_EQEP_QCLR_RESERVED_1_MAX                                           (0x0007U)

#define CSL_EQEP_QCLR_RESETVAL                                                 (0x0000U)

/* QFRC */

#define CSL_EQEP_QFRC_RESERVED_1_MASK                                          (0x0001U)
#define CSL_EQEP_QFRC_RESERVED_1_SHIFT                                         (0x0000U)
#define CSL_EQEP_QFRC_RESERVED_1_RESETVAL                                      (0x0000U)
#define CSL_EQEP_QFRC_RESERVED_1_MAX                                           (0x0001U)

#define CSL_EQEP_QFRC_PCE_MASK                                                 (0x0002U)
#define CSL_EQEP_QFRC_PCE_SHIFT                                                (0x0001U)
#define CSL_EQEP_QFRC_PCE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PCE_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PCE_VAL_PCE_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PCE_VAL_PCE_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_PHE_MASK                                                 (0x0004U)
#define CSL_EQEP_QFRC_PHE_SHIFT                                                (0x0002U)
#define CSL_EQEP_QFRC_PHE_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PHE_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PHE_VAL_PHE_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PHE_VAL_PHE_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_QDC_MASK                                                 (0x0008U)
#define CSL_EQEP_QFRC_QDC_SHIFT                                                (0x0003U)
#define CSL_EQEP_QFRC_QDC_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_QDC_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_QDC_VAL_QDC_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_QDC_VAL_QDC_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_WTO_MASK                                                 (0x0010U)
#define CSL_EQEP_QFRC_WTO_SHIFT                                                (0x0004U)
#define CSL_EQEP_QFRC_WTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_WTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_WTO_VAL_WTO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_WTO_VAL_WTO_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_PCU_MASK                                                 (0x0020U)
#define CSL_EQEP_QFRC_PCU_SHIFT                                                (0x0005U)
#define CSL_EQEP_QFRC_PCU_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PCU_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PCU_VAL_PCU_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PCU_VAL_PCU_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_PCO_MASK                                                 (0x0040U)
#define CSL_EQEP_QFRC_PCO_SHIFT                                                (0x0006U)
#define CSL_EQEP_QFRC_PCO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PCO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PCO_VAL_PCO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PCO_VAL_PCO_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_PCR_MASK                                                 (0x0080U)
#define CSL_EQEP_QFRC_PCR_SHIFT                                                (0x0007U)
#define CSL_EQEP_QFRC_PCR_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PCR_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PCR_VAL_PCR_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PCR_VAL_PCR_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_PCM_MASK                                                 (0x0100U)
#define CSL_EQEP_QFRC_PCM_SHIFT                                                (0x0008U)
#define CSL_EQEP_QFRC_PCM_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_PCM_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_PCM_VAL_PCM_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_PCM_VAL_PCM_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_SEL_MASK                                                 (0x0200U)
#define CSL_EQEP_QFRC_SEL_SHIFT                                                (0x0009U)
#define CSL_EQEP_QFRC_SEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_SEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_SEL_VAL_SEL_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_SEL_VAL_SEL_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_IEL_MASK                                                 (0x0400U)
#define CSL_EQEP_QFRC_IEL_SHIFT                                                (0x000AU)
#define CSL_EQEP_QFRC_IEL_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_IEL_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_IEL_VAL_IEL_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_IEL_VAL_IEL_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_UTO_MASK                                                 (0x0800U)
#define CSL_EQEP_QFRC_UTO_SHIFT                                                (0x000BU)
#define CSL_EQEP_QFRC_UTO_RESETVAL                                             (0x0000U)
#define CSL_EQEP_QFRC_UTO_MAX                                                  (0x0001U)

#define CSL_EQEP_QFRC_UTO_VAL_UTO_NOEFFECT                                     (0x0U)
#define CSL_EQEP_QFRC_UTO_VAL_UTO_FORCE                                        (0x1U)

#define CSL_EQEP_QFRC_QMAE_MASK                                                (0x1000U)
#define CSL_EQEP_QFRC_QMAE_SHIFT                                               (0x000CU)
#define CSL_EQEP_QFRC_QMAE_RESETVAL                                            (0x0000U)
#define CSL_EQEP_QFRC_QMAE_MAX                                                 (0x0001U)

#define CSL_EQEP_QFRC_QMAE_VAL_QMAE_NOEFFECT                                   (0x0U)
#define CSL_EQEP_QFRC_QMAE_VAL_QMAE_FORCE                                      (0x1U)

#define CSL_EQEP_QFRC_RESERVED_2_MASK                                          (0xE000U)
#define CSL_EQEP_QFRC_RESERVED_2_SHIFT                                         (0x000DU)
#define CSL_EQEP_QFRC_RESERVED_2_RESETVAL                                      (0x0000U)
#define CSL_EQEP_QFRC_RESERVED_2_MAX                                           (0x0007U)

#define CSL_EQEP_QFRC_RESETVAL                                                 (0x0000U)

/* QEPSTS */

#define CSL_EQEP_QEPSTS_PCEF_MASK                                              (0x0001U)
#define CSL_EQEP_QEPSTS_PCEF_SHIFT                                             (0x0000U)
#define CSL_EQEP_QEPSTS_PCEF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_PCEF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_PCEF_VAL_PCEF_NOERROR                                  (0x0U)
#define CSL_EQEP_QEPSTS_PCEF_VAL_PCEF_ERROR                                    (0x1U)

#define CSL_EQEP_QEPSTS_FIMF_MASK                                              (0x0002U)
#define CSL_EQEP_QEPSTS_FIMF_SHIFT                                             (0x0001U)
#define CSL_EQEP_QEPSTS_FIMF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_FIMF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_FIMF_VAL_FIMF_WRT1                                     (0x0U)
#define CSL_EQEP_QEPSTS_FIMF_VAL_FIMF_SETINDEX                                 (0x1U)

#define CSL_EQEP_QEPSTS_CDEF_MASK                                              (0x0004U)
#define CSL_EQEP_QEPSTS_CDEF_SHIFT                                             (0x0002U)
#define CSL_EQEP_QEPSTS_CDEF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_CDEF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_CDEF_VAL_CDEF_WRT1                                     (0x0U)
#define CSL_EQEP_QEPSTS_CDEF_VAL_CDEF_DIRECT                                   (0x1U)

#define CSL_EQEP_QEPSTS_COEF_MASK                                              (0x0008U)
#define CSL_EQEP_QEPSTS_COEF_SHIFT                                             (0x0003U)
#define CSL_EQEP_QEPSTS_COEF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_COEF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_COEF_VAL_COEF_WRT1                                     (0x0U)
#define CSL_EQEP_QEPSTS_COEF_VAL_COEF_OVF                                      (0x1U)

#define CSL_EQEP_QEPSTS_QDLF_MASK                                              (0x0010U)
#define CSL_EQEP_QEPSTS_QDLF_SHIFT                                             (0x0004U)
#define CSL_EQEP_QEPSTS_QDLF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_QDLF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_QDLF_VAL_QDLF_COUNTERCLK                               (0x0U)
#define CSL_EQEP_QEPSTS_QDLF_VAL_QDLF_CLK                                      (0x1U)

#define CSL_EQEP_QEPSTS_QDF_MASK                                               (0x0020U)
#define CSL_EQEP_QEPSTS_QDF_SHIFT                                              (0x0005U)
#define CSL_EQEP_QEPSTS_QDF_RESETVAL                                           (0x0000U)
#define CSL_EQEP_QEPSTS_QDF_MAX                                                (0x0001U)

#define CSL_EQEP_QEPSTS_QDF_VAL_QDF_COUNTERCLK                                 (0x0U)
#define CSL_EQEP_QEPSTS_QDF_VAL_QDF_CLK                                        (0x1U)

#define CSL_EQEP_QEPSTS_FIDF_MASK                                              (0x0040U)
#define CSL_EQEP_QEPSTS_FIDF_SHIFT                                             (0x0006U)
#define CSL_EQEP_QEPSTS_FIDF_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QEPSTS_FIDF_MAX                                               (0x0001U)

#define CSL_EQEP_QEPSTS_FIDF_VAL_FIDF_COUNTERCLK                               (0x0U)
#define CSL_EQEP_QEPSTS_FIDF_VAL_FIDF_CLK                                      (0x1U)

#define CSL_EQEP_QEPSTS_UPEVNT_MASK                                            (0x0080U)
#define CSL_EQEP_QEPSTS_UPEVNT_SHIFT                                           (0x0007U)
#define CSL_EQEP_QEPSTS_UPEVNT_RESETVAL                                        (0x0001U)
#define CSL_EQEP_QEPSTS_UPEVNT_MAX                                             (0x0001U)

#define CSL_EQEP_QEPSTS_UPEVNT_VAL_UPEVNT_NODETCT                              (0x0U)
#define CSL_EQEP_QEPSTS_UPEVNT_VAL_UPEVNT_DETCT                                (0x1U)

#define CSL_EQEP_QEPSTS_RESERVED_1_MASK                                        (0xFF00U)
#define CSL_EQEP_QEPSTS_RESERVED_1_SHIFT                                       (0x0008U)
#define CSL_EQEP_QEPSTS_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_EQEP_QEPSTS_RESERVED_1_MAX                                         (0x00FFU)

#define CSL_EQEP_QEPSTS_RESETVAL                                               (0x0080U)

/* QCTMR */

#define CSL_EQEP_QCTMR_QCTMR_MASK                                              (0xFFFFU)
#define CSL_EQEP_QCTMR_QCTMR_SHIFT                                             (0x0000U)
#define CSL_EQEP_QCTMR_QCTMR_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QCTMR_QCTMR_MAX                                               (0xFFFFU)

#define CSL_EQEP_QCTMR_RESETVAL                                                (0x0000U)

/* QCPRD */

#define CSL_EQEP_QCPRD_QCPRD_MASK                                              (0xFFFFU)
#define CSL_EQEP_QCPRD_QCPRD_SHIFT                                             (0x0000U)
#define CSL_EQEP_QCPRD_QCPRD_RESETVAL                                          (0x0000U)
#define CSL_EQEP_QCPRD_QCPRD_MAX                                               (0xFFFFU)

#define CSL_EQEP_QCPRD_RESETVAL                                                (0x0000U)

/* QCTMRLAT */

#define CSL_EQEP_QCTMRLAT_QCTMRLAT_MASK                                        (0xFFFFU)
#define CSL_EQEP_QCTMRLAT_QCTMRLAT_SHIFT                                       (0x0000U)
#define CSL_EQEP_QCTMRLAT_QCTMRLAT_RESETVAL                                    (0x0000U)
#define CSL_EQEP_QCTMRLAT_QCTMRLAT_MAX                                         (0xFFFFU)

#define CSL_EQEP_QCTMRLAT_RESETVAL                                             (0x0000U)

/* QCPRDLAT */

#define CSL_EQEP_QCPRDLAT_QCPRDLAT_MASK                                        (0xFFFFU)
#define CSL_EQEP_QCPRDLAT_QCPRDLAT_SHIFT                                       (0x0000U)
#define CSL_EQEP_QCPRDLAT_QCPRDLAT_RESETVAL                                    (0x0000U)
#define CSL_EQEP_QCPRDLAT_QCPRDLAT_MAX                                         (0xFFFFU)

#define CSL_EQEP_QCPRDLAT_RESETVAL                                             (0x0000U)

/* REV */

#define CSL_EQEP_REV_MAJOR_MASK                                                (0x00000007U)
#define CSL_EQEP_REV_MAJOR_SHIFT                                               (0x00000000U)
#define CSL_EQEP_REV_MAJOR_RESETVAL                                            (0x00000001U)
#define CSL_EQEP_REV_MAJOR_MAX                                                 (0x00000007U)

#define CSL_EQEP_REV_MINOR_MASK                                                (0x00000038U)
#define CSL_EQEP_REV_MINOR_SHIFT                                               (0x00000003U)
#define CSL_EQEP_REV_MINOR_RESETVAL                                            (0x00000002U)
#define CSL_EQEP_REV_MINOR_MAX                                                 (0x00000007U)

#define CSL_EQEP_REV_RESERVED_1_MASK                                           (0xFFFFFFC0U)
#define CSL_EQEP_REV_RESERVED_1_SHIFT                                          (0x00000006U)
#define CSL_EQEP_REV_RESERVED_1_RESETVAL                                       (0x00000000U)
#define CSL_EQEP_REV_RESERVED_1_MAX                                            (0x03FFFFFFU)

#define CSL_EQEP_REV_RESETVAL                                                  (0x00000011U)

/* QEPSTROBESEL */

#define CSL_EQEP_QEPSTROBESEL_STROBESEL_MASK                                   (0x00000003U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_SHIFT                                  (0x00000000U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_RESETVAL                               (0x00000000U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_MAX                                    (0x00000003U)

#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_QS_AFTER_POL_MUX                   (0x1U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_QS_AFTER_POL_MUX                   (0x1U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_ADCSOCA_AS_QS                      (0x2U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_ADCSOCB_AS_QS                      (0x3U)

#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_MASK                                  (0xFFFFFFFCU)
#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_SHIFT                                 (0x00000002U)
#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_MAX                                   (0x3FFFFFFFU)

#define CSL_EQEP_QEPSTROBESEL_RESETVAL                                         (0x00000000U)

/* QMACTRL */

#define CSL_EQEP_QMACTRL_MODE_MASK                                             (0x00000007U)
#define CSL_EQEP_QMACTRL_MODE_SHIFT                                            (0x00000000U)
#define CSL_EQEP_QMACTRL_MODE_RESETVAL                                         (0x00000000U)
#define CSL_EQEP_QMACTRL_MODE_MAX                                              (0x00000007U)

#define CSL_EQEP_QMACTRL_RESERVED_1_MASK                                       (0xFFFFFFF8U)
#define CSL_EQEP_QMACTRL_RESERVED_1_SHIFT                                      (0x00000003U)
#define CSL_EQEP_QMACTRL_RESERVED_1_RESETVAL                                   (0x00000000U)
#define CSL_EQEP_QMACTRL_RESERVED_1_MAX                                        (0x1FFFFFFFU)

#define CSL_EQEP_QMACTRL_RESETVAL                                              (0x00000000U)

/* QEPSRCSEL */

#define CSL_EQEP_QEPSRCSEL_QEPASEL_MASK                                        (0x0000001FU)
#define CSL_EQEP_QEPSRCSEL_QEPASEL_SHIFT                                       (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPASEL_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPASEL_MAX                                         (0x0000001FU)

#define CSL_EQEP_QEPSRCSEL_RESERVED_1_MASK                                     (0x000000E0U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_1_SHIFT                                    (0x00000005U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_1_MAX                                      (0x00000007U)

#define CSL_EQEP_QEPSRCSEL_QEPBSEL_MASK                                        (0x00001F00U)
#define CSL_EQEP_QEPSRCSEL_QEPBSEL_SHIFT                                       (0x00000008U)
#define CSL_EQEP_QEPSRCSEL_QEPBSEL_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPBSEL_MAX                                         (0x0000001FU)

#define CSL_EQEP_QEPSRCSEL_RESERVED_2_MASK                                     (0x0000E000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_2_SHIFT                                    (0x0000000DU)
#define CSL_EQEP_QEPSRCSEL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_2_MAX                                      (0x00000007U)

#define CSL_EQEP_QEPSRCSEL_QEPISEL_MASK                                        (0x001F0000U)
#define CSL_EQEP_QEPSRCSEL_QEPISEL_SHIFT                                       (0x00000010U)
#define CSL_EQEP_QEPSRCSEL_QEPISEL_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPISEL_MAX                                         (0x0000001FU)

#define CSL_EQEP_QEPSRCSEL_RESERVED_3_MASK                                     (0x00E00000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_3_SHIFT                                    (0x00000015U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_3_RESETVAL                                 (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_3_MAX                                      (0x00000007U)

#define CSL_EQEP_QEPSRCSEL_QEPSSEL_MASK                                        (0x1F000000U)
#define CSL_EQEP_QEPSRCSEL_QEPSSEL_SHIFT                                       (0x00000018U)
#define CSL_EQEP_QEPSRCSEL_QEPSSEL_RESETVAL                                    (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPSSEL_MAX                                         (0x0000001FU)

#define CSL_EQEP_QEPSRCSEL_RESERVED_4_MASK                                     (0xE0000000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_4_SHIFT                                    (0x0000001DU)
#define CSL_EQEP_QEPSRCSEL_RESERVED_4_RESETVAL                                 (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_4_MAX                                      (0x00000007U)

#define CSL_EQEP_QEPSRCSEL_RESETVAL                                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
