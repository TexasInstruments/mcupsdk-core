/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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

#ifndef CSLR_EQEP_H_
#define CSLR_EQEP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_EQEP_REGS_BASE                                              (0x00000000U)


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
    volatile uint16_t QDECCTL_TYPE2;
    volatile uint16_t QEPCTL;
    volatile uint16_t QCAPCTL;
    volatile uint16_t QPOSCTL;
    volatile uint16_t QEINT_TYPE1;
    volatile uint16_t QFLG_TYPE1;
    volatile uint16_t QCLR_TYPE1;
    volatile uint16_t QFRC_TYPE1;
    volatile uint16_t QEPSTS_TYPE1;
    volatile uint16_t QCTMR;
    volatile uint16_t QCPRD;
    volatile uint16_t QCTMRLAT;
    volatile uint16_t QCPRDLAT;
    volatile uint16_t RESERVED_1[13];
    volatile uint32_t PID;                       /* Peripheral ID Register */
    volatile uint32_t REV_TYPE2;
    volatile uint32_t QEPSTROBESEL;
    volatile uint32_t QMACTRL;
    volatile uint32_t QEPSRCSEL;
    volatile uint16_t RESERVED_2[8];
} CSL_eqep_Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_EQEP_QPOSCNT                                                (0x00000000U)
#define CSL_EQEP_QPOSINIT                                               (0x00000004U)
#define CSL_EQEP_QPOSMAX                                                (0x00000008U)
#define CSL_EQEP_QPOSCMP                                                (0x0000000CU)
#define CSL_EQEP_QPOSILAT                                               (0x00000010U)
#define CSL_EQEP_QPOSSLAT                                               (0x00000014U)
#define CSL_EQEP_QPOSLAT                                                (0x00000018U)
#define CSL_EQEP_QUTMR                                                  (0x0000001CU)
#define CSL_EQEP_QUPRD                                                  (0x00000020U)
#define CSL_EQEP_QWDTMR                                                 (0x00000024U)
#define CSL_EQEP_QWDPRD                                                 (0x00000026U)
#define CSL_EQEP_QDECCTL_TYPE2                                          (0x00000028U)
#define CSL_EQEP_QEPCTL                                                 (0x0000002AU)
#define CSL_EQEP_QCAPCTL                                                (0x0000002CU)
#define CSL_EQEP_QPOSCTL                                                (0x0000002EU)
#define CSL_EQEP_QEINT_TYPE1                                            (0x00000030U)
#define CSL_EQEP_QFLG_TYPE1                                             (0x00000032U)
#define CSL_EQEP_QCLR_TYPE1                                             (0x00000034U)
#define CSL_EQEP_QFRC_TYPE1                                             (0x00000036U)
#define CSL_EQEP_QEPSTS_TYPE1                                           (0x00000038U)
#define CSL_EQEP_QCTMR                                                  (0x0000003AU)
#define CSL_EQEP_QCPRD                                                  (0x0000003CU)
#define CSL_EQEP_QCTMRLAT                                               (0x0000003EU)
#define CSL_EQEP_QCPRDLAT                                               (0x00000040U)
#define CSL_EQEP_RESERVED_1(RESERVED_1)                                 (0x00000042U+((RESERVED_1)*0x2U))
#define CSL_EQEP_PID                                                    (0x0000005CU)
#define CSL_EQEP_REV_TYPE2                                              (0x00000060U)
#define CSL_EQEP_QEPSTROBESEL                                           (0x00000064U)
#define CSL_EQEP_QMACTRL                                                (0x00000068U)
#define CSL_EQEP_QEPSRCSEL                                              (0x0000006CU)
#define CSL_EQEP_RESERVED_2(RESERVED_2)                                 (0x00000070U+((RESERVED_2)*0x2U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* QPOSCNT */

#define CSL_EQEP_QPOSCNT_QPOSCNT_MASK                                   (0xFFFFFFFFU)
#define CSL_EQEP_QPOSCNT_QPOSCNT_SHIFT                                  (0x00000000U)
#define CSL_EQEP_QPOSCNT_QPOSCNT_MAX                                    (0xFFFFFFFFU)

/* QPOSINIT */

#define CSL_EQEP_QPOSINIT_QPOSINIT_MASK                                 (0xFFFFFFFFU)
#define CSL_EQEP_QPOSINIT_QPOSINIT_SHIFT                                (0x00000000U)
#define CSL_EQEP_QPOSINIT_QPOSINIT_MAX                                  (0xFFFFFFFFU)

/* QPOSMAX */

#define CSL_EQEP_QPOSMAX_QPOSMAX_MASK                                   (0xFFFFFFFFU)
#define CSL_EQEP_QPOSMAX_QPOSMAX_SHIFT                                  (0x00000000U)
#define CSL_EQEP_QPOSMAX_QPOSMAX_MAX                                    (0xFFFFFFFFU)

/* QPOSCMP */

#define CSL_EQEP_QPOSCMP_QPOSCMP_MASK                                   (0xFFFFFFFFU)
#define CSL_EQEP_QPOSCMP_QPOSCMP_SHIFT                                  (0x00000000U)
#define CSL_EQEP_QPOSCMP_QPOSCMP_MAX                                    (0xFFFFFFFFU)

/* QPOSILAT */

#define CSL_EQEP_QPOSILAT_QPOSILAT_MASK                                 (0xFFFFFFFFU)
#define CSL_EQEP_QPOSILAT_QPOSILAT_SHIFT                                (0x00000000U)
#define CSL_EQEP_QPOSILAT_QPOSILAT_MAX                                  (0xFFFFFFFFU)

/* QPOSSLAT */

#define CSL_EQEP_QPOSSLAT_QPOSSLAT_MASK                                 (0xFFFFFFFFU)
#define CSL_EQEP_QPOSSLAT_QPOSSLAT_SHIFT                                (0x00000000U)
#define CSL_EQEP_QPOSSLAT_QPOSSLAT_MAX                                  (0xFFFFFFFFU)

/* QPOSLAT */

#define CSL_EQEP_QPOSLAT_QPOSLAT_MASK                                   (0xFFFFFFFFU)
#define CSL_EQEP_QPOSLAT_QPOSLAT_SHIFT                                  (0x00000000U)
#define CSL_EQEP_QPOSLAT_QPOSLAT_MAX                                    (0xFFFFFFFFU)

/* QUTMR */

#define CSL_EQEP_QUTMR_QUTMR_MASK                                       (0xFFFFFFFFU)
#define CSL_EQEP_QUTMR_QUTMR_SHIFT                                      (0x00000000U)
#define CSL_EQEP_QUTMR_QUTMR_MAX                                        (0xFFFFFFFFU)

/* QUPRD */

#define CSL_EQEP_QUPRD_QUPRD_MASK                                       (0xFFFFFFFFU)
#define CSL_EQEP_QUPRD_QUPRD_SHIFT                                      (0x00000000U)
#define CSL_EQEP_QUPRD_QUPRD_MAX                                        (0xFFFFFFFFU)

/* QWDTMR */

#define CSL_EQEP_QWDTMR_QWDTMR_MASK                                     (0xFFFFU)
#define CSL_EQEP_QWDTMR_QWDTMR_SHIFT                                    (0x0000U)
#define CSL_EQEP_QWDTMR_QWDTMR_MAX                                      (0xFFFFU)

/* QWDPRD */

#define CSL_EQEP_QWDPRD_QWDPRD_MASK                                     (0xFFFFU)
#define CSL_EQEP_QWDPRD_QWDPRD_SHIFT                                    (0x0000U)
#define CSL_EQEP_QWDPRD_QWDPRD_MAX                                      (0xFFFFU)

/* QDECCTL_TYPE2 */

#define CSL_EQEP_QDECCTL_TYPE2_QIDIRE_MASK                              (0x0001U)
#define CSL_EQEP_QDECCTL_TYPE2_QIDIRE_SHIFT                             (0x0000U)
#define CSL_EQEP_QDECCTL_TYPE2_QIDIRE_MAX                               (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_RESERVED_1_MASK                          (0x001EU)
#define CSL_EQEP_QDECCTL_TYPE2_RESERVED_1_SHIFT                         (0x0001U)
#define CSL_EQEP_QDECCTL_TYPE2_RESERVED_1_MAX                           (0x000FU)

#define CSL_EQEP_QDECCTL_TYPE2_QSP_MASK                                 (0x0020U)
#define CSL_EQEP_QDECCTL_TYPE2_QSP_SHIFT                                (0x0005U)
#define CSL_EQEP_QDECCTL_TYPE2_QSP_MAX                                  (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_QSP_VAL_QSP_NOPOLAR                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_QSP_VAL_QSP_POLAR                        (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_QIP_MASK                                 (0x0040U)
#define CSL_EQEP_QDECCTL_TYPE2_QIP_SHIFT                                (0x0006U)
#define CSL_EQEP_QDECCTL_TYPE2_QIP_MAX                                  (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_QIP_VAL_QIP_NOPOLAR                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_QIP_VAL_QIP_POLAR                        (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_QBP_MASK                                 (0x0080U)
#define CSL_EQEP_QDECCTL_TYPE2_QBP_SHIFT                                (0x0007U)
#define CSL_EQEP_QDECCTL_TYPE2_QBP_MAX                                  (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_QBP_VAL_QBP_NOPOLAR                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_QBP_VAL_QBP_POLAR                        (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_QAP_MASK                                 (0x0100U)
#define CSL_EQEP_QDECCTL_TYPE2_QAP_SHIFT                                (0x0008U)
#define CSL_EQEP_QDECCTL_TYPE2_QAP_MAX                                  (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_QAP_VAL_QAP_NOPOLAR                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_QAP_VAL_QAP_POLAR                        (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_IGATE_MASK                               (0x0200U)
#define CSL_EQEP_QDECCTL_TYPE2_IGATE_SHIFT                              (0x0009U)
#define CSL_EQEP_QDECCTL_TYPE2_IGATE_MAX                                (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_IGATE_VAL_IGATE_DISABLE                  (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_IGATE_VAL_IGATE_ENABLE                   (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_SWAP_MASK                                (0x0400U)
#define CSL_EQEP_QDECCTL_TYPE2_SWAP_SHIFT                               (0x000AU)
#define CSL_EQEP_QDECCTL_TYPE2_SWAP_MAX                                 (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_SWAP_VAL_SWAP_DISABLE                    (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_SWAP_VAL_SWAP_ENABLE                     (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_XCR_MASK                                 (0x0800U)
#define CSL_EQEP_QDECCTL_TYPE2_XCR_SHIFT                                (0x000BU)
#define CSL_EQEP_QDECCTL_TYPE2_XCR_MAX                                  (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_XCR_VAL_XCR_2XRESOL                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_XCR_VAL_XCR_1XRESOL                      (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_SPSEL_MASK                               (0x1000U)
#define CSL_EQEP_QDECCTL_TYPE2_SPSEL_SHIFT                              (0x000CU)
#define CSL_EQEP_QDECCTL_TYPE2_SPSEL_MAX                                (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_SPSEL_VAL_INDEX_PIN                      (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_SPSEL_VAL_STROBE_PIN                     (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_SOEN_MASK                                (0x2000U)
#define CSL_EQEP_QDECCTL_TYPE2_SOEN_SHIFT                               (0x000DU)
#define CSL_EQEP_QDECCTL_TYPE2_SOEN_MAX                                 (0x0001U)

#define CSL_EQEP_QDECCTL_TYPE2_SOEN_VAL_SYNC_DISABLE                    (0x0U)
#define CSL_EQEP_QDECCTL_TYPE2_SOEN_VAL_SYNC_ENABLE                     (0x1U)

#define CSL_EQEP_QDECCTL_TYPE2_QSRC_MASK                                (0xC000U)
#define CSL_EQEP_QDECCTL_TYPE2_QSRC_SHIFT                               (0x000EU)
#define CSL_EQEP_QDECCTL_TYPE2_QSRC_MAX                                 (0x0003U)

/* QEPCTL */

#define CSL_EQEP_QEPCTL_WDE_MASK                                        (0x0001U)
#define CSL_EQEP_QEPCTL_WDE_SHIFT                                       (0x0000U)
#define CSL_EQEP_QEPCTL_WDE_MAX                                         (0x0001U)

#define CSL_EQEP_QEPCTL_WDE_VAL_WDE_DISABLE                             (0x0U)
#define CSL_EQEP_QEPCTL_WDE_VAL_WDE_ENABLE                              (0x1U)

#define CSL_EQEP_QEPCTL_UTE_MASK                                        (0x0002U)
#define CSL_EQEP_QEPCTL_UTE_SHIFT                                       (0x0001U)
#define CSL_EQEP_QEPCTL_UTE_MAX                                         (0x0001U)

#define CSL_EQEP_QEPCTL_UTE_VAL_UTE_DISABLE                             (0x0U)
#define CSL_EQEP_QEPCTL_UTE_VAL_UTE_ENABLE                              (0x1U)

#define CSL_EQEP_QEPCTL_QCLM_MASK                                       (0x0004U)
#define CSL_EQEP_QEPCTL_QCLM_SHIFT                                      (0x0002U)
#define CSL_EQEP_QEPCTL_QCLM_MAX                                        (0x0001U)

#define CSL_EQEP_QEPCTL_QCLM_VAL_QCLM_CPU                               (0x0U)
#define CSL_EQEP_QEPCTL_QCLM_VAL_QCLM_TIMEOUT                           (0x1U)

#define CSL_EQEP_QEPCTL_QPEN_MASK                                       (0x0008U)
#define CSL_EQEP_QEPCTL_QPEN_SHIFT                                      (0x0003U)
#define CSL_EQEP_QEPCTL_QPEN_MAX                                        (0x0001U)

#define CSL_EQEP_QEPCTL_QPEN_VAL_QPEN_RESET                             (0x0U)
#define CSL_EQEP_QEPCTL_QPEN_VAL_QPEN_ENABLE                            (0x1U)

#define CSL_EQEP_QEPCTL_IEL_MASK                                        (0x0030U)
#define CSL_EQEP_QEPCTL_IEL_SHIFT                                       (0x0004U)
#define CSL_EQEP_QEPCTL_IEL_MAX                                         (0x0003U)

#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_RSVD                                (0x0U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_POSRISING                           (0x1U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_POSFALLING                          (0x2U)
#define CSL_EQEP_QEPCTL_IEL_VAL_IEL_SIM                                 (0x3U)

#define CSL_EQEP_QEPCTL_SEL_MASK                                        (0x0040U)
#define CSL_EQEP_QEPCTL_SEL_SHIFT                                       (0x0006U)
#define CSL_EQEP_QEPCTL_SEL_MAX                                         (0x0001U)

#define CSL_EQEP_QEPCTL_SEL_VAL_SEL_QEPSRISING                          (0x0U)
#define CSL_EQEP_QEPCTL_SEL_VAL_SEL_QEPSCLOCK                           (0x1U)

#define CSL_EQEP_QEPCTL_SWI_MASK                                        (0x0080U)
#define CSL_EQEP_QEPCTL_SWI_SHIFT                                       (0x0007U)
#define CSL_EQEP_QEPCTL_SWI_MAX                                         (0x0001U)

#define CSL_EQEP_QEPCTL_SWI_VAL_SWI_NOTHING                             (0x0U)
#define CSL_EQEP_QEPCTL_SWI_VAL_SWI_INITPOS                             (0x1U)

#define CSL_EQEP_QEPCTL_IEI_MASK                                        (0x0300U)
#define CSL_EQEP_QEPCTL_IEI_SHIFT                                       (0x0008U)
#define CSL_EQEP_QEPCTL_IEI_MAX                                         (0x0003U)

#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_NOTHING0                            (0x0U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_NOTHING1                            (0x1U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_INITRISING                          (0x2U)
#define CSL_EQEP_QEPCTL_IEI_VAL_IEI_INITFALLING                         (0x3U)

#define CSL_EQEP_QEPCTL_SEI_MASK                                        (0x0C00U)
#define CSL_EQEP_QEPCTL_SEI_SHIFT                                       (0x000AU)
#define CSL_EQEP_QEPCTL_SEI_MAX                                         (0x0003U)

#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_NOTHING0                            (0x0U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_NOTHING1                            (0x1U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_INITQEPSRISING                      (0x2U)
#define CSL_EQEP_QEPCTL_SEI_VAL_SEI_INITQEPSCLOCK                       (0x3U)

#define CSL_EQEP_QEPCTL_PCRM_MASK                                       (0x3000U)
#define CSL_EQEP_QEPCTL_PCRM_SHIFT                                      (0x000CU)
#define CSL_EQEP_QEPCTL_PCRM_MAX                                        (0x0003U)

#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_INDEX                             (0x0U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_MAXPOS                            (0x1U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_FIRSTINDEX                        (0x2U)
#define CSL_EQEP_QEPCTL_PCRM_VAL_PCRM_TIMEEVENT                         (0x3U)

#define CSL_EQEP_QEPCTL_FREE_SOFT_MASK                                  (0xC000U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_SHIFT                                 (0x000EU)
#define CSL_EQEP_QEPCTL_FREE_SOFT_MAX                                   (0x0003U)

#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_0                       (0x0U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_1                       (0x1U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_2                       (0x2U)
#define CSL_EQEP_QEPCTL_FREE_SOFT_VAL_FREE_SOFT_3                       (0x3U)

/* QCAPCTL */

#define CSL_EQEP_QCAPCTL_UPPS_MASK                                      (0x000FU)
#define CSL_EQEP_QCAPCTL_UPPS_SHIFT                                     (0x0000U)
#define CSL_EQEP_QCAPCTL_UPPS_MAX                                       (0x000FU)

#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK1                                 (0x0U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK2                                 (0x1U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK4                                 (0x2U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK8                                 (0x3U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK16                                (0x4U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK32                                (0x5U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK64                                (0x6U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK128                               (0x7U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK256                               (0x8U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK512                               (0x9U)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK1024                              (0xAU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK2048                              (0xBU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD0                            (0xCU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD1                            (0xDU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD2                            (0xEU)
#define CSL_EQEP_QCAPCTL_UPPS_VAL_QCLK_RSVD3                            (0xFU)

#define CSL_EQEP_QCAPCTL_CCPS_MASK                                      (0x0070U)
#define CSL_EQEP_QCAPCTL_CCPS_SHIFT                                     (0x0004U)
#define CSL_EQEP_QCAPCTL_CCPS_MAX                                       (0x0007U)

#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT1                            (0x0U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT2                            (0x1U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT4                            (0x2U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT8                            (0x3U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT16                           (0x4U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT32                           (0x5U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT64                           (0x6U)
#define CSL_EQEP_QCAPCTL_CCPS_VAL_SYSCLKOUT128                          (0x7U)

#define CSL_EQEP_QCAPCTL_RESERVED_1_MASK                                (0x7F80U)
#define CSL_EQEP_QCAPCTL_RESERVED_1_SHIFT                               (0x0007U)
#define CSL_EQEP_QCAPCTL_RESERVED_1_MAX                                 (0x00FFU)

#define CSL_EQEP_QCAPCTL_CEN_MASK                                       (0x8000U)
#define CSL_EQEP_QCAPCTL_CEN_SHIFT                                      (0x000FU)
#define CSL_EQEP_QCAPCTL_CEN_MAX                                        (0x0001U)

#define CSL_EQEP_QCAPCTL_CEN_VAL_CEN_DISABLE                            (0x0U)
#define CSL_EQEP_QCAPCTL_CEN_VAL_CEN_ENABLE                             (0x1U)

/* QPOSCTL */

#define CSL_EQEP_QPOSCTL_PCSPW_MASK                                     (0x0FFFU)
#define CSL_EQEP_QPOSCTL_PCSPW_SHIFT                                    (0x0000U)
#define CSL_EQEP_QPOSCTL_PCSPW_MAX                                      (0x0FFFU)

#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT4                           (0x0U)
#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT8                           (0x1U)
#define CSL_EQEP_QPOSCTL_PCSPW_VAL_SYSCLKOUT16384                       (0xFFFU)

#define CSL_EQEP_QPOSCTL_PCE_MASK                                       (0x1000U)
#define CSL_EQEP_QPOSCTL_PCE_SHIFT                                      (0x000CU)
#define CSL_EQEP_QPOSCTL_PCE_MAX                                        (0x0001U)

#define CSL_EQEP_QPOSCTL_PCE_VAL_PCE_DISABLE                            (0x0U)
#define CSL_EQEP_QPOSCTL_PCE_VAL_PCE_ENABLE                             (0x1U)

#define CSL_EQEP_QPOSCTL_PCPOL_MASK                                     (0x2000U)
#define CSL_EQEP_QPOSCTL_PCPOL_SHIFT                                    (0x000DU)
#define CSL_EQEP_QPOSCTL_PCPOL_MAX                                      (0x0001U)

#define CSL_EQEP_QPOSCTL_PCPOL_VAL_PCPOL_HIGH                           (0x0U)
#define CSL_EQEP_QPOSCTL_PCPOL_VAL_PCPOL_LOW                            (0x1U)

#define CSL_EQEP_QPOSCTL_PCLOAD_MASK                                    (0x4000U)
#define CSL_EQEP_QPOSCTL_PCLOAD_SHIFT                                   (0x000EU)
#define CSL_EQEP_QPOSCTL_PCLOAD_MAX                                     (0x0001U)

#define CSL_EQEP_QPOSCTL_PCLOAD_VAL_PCLOAD_0                            (0x0U)
#define CSL_EQEP_QPOSCTL_PCLOAD_VAL_PCLOAD_QPOSCMP                      (0x1U)

#define CSL_EQEP_QPOSCTL_PCSHDW_MASK                                    (0x8000U)
#define CSL_EQEP_QPOSCTL_PCSHDW_SHIFT                                   (0x000FU)
#define CSL_EQEP_QPOSCTL_PCSHDW_MAX                                     (0x0001U)

#define CSL_EQEP_QPOSCTL_PCSHDW_VAL_PCSHDW_DISABLE                      (0x0U)
#define CSL_EQEP_QPOSCTL_PCSHDW_VAL_PCSHDW_ENABLE                       (0x1U)

/* QEINT_TYPE1 */

#define CSL_EQEP_QEINT_TYPE1_RESERVED_1_MASK                            (0x0001U)
#define CSL_EQEP_QEINT_TYPE1_RESERVED_1_SHIFT                           (0x0000U)
#define CSL_EQEP_QEINT_TYPE1_RESERVED_1_MAX                             (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCE_MASK                                   (0x0002U)
#define CSL_EQEP_QEINT_TYPE1_PCE_SHIFT                                  (0x0001U)
#define CSL_EQEP_QEINT_TYPE1_PCE_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCE_VAL_PCE_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_PCE_VAL_PCE_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_QPE_MASK                                   (0x0004U)
#define CSL_EQEP_QEINT_TYPE1_QPE_SHIFT                                  (0x0002U)
#define CSL_EQEP_QEINT_TYPE1_QPE_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_QPE_VAL_QPE_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_QPE_VAL_QPE_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_QDC_MASK                                   (0x0008U)
#define CSL_EQEP_QEINT_TYPE1_QDC_SHIFT                                  (0x0003U)
#define CSL_EQEP_QEINT_TYPE1_QDC_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_QDC_VAL_QDC_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_QDC_VAL_QDC_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_WTO_MASK                                   (0x0010U)
#define CSL_EQEP_QEINT_TYPE1_WTO_SHIFT                                  (0x0004U)
#define CSL_EQEP_QEINT_TYPE1_WTO_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_WTO_VAL_WTO_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_WTO_VAL_WTO_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_PCU_MASK                                   (0x0020U)
#define CSL_EQEP_QEINT_TYPE1_PCU_SHIFT                                  (0x0005U)
#define CSL_EQEP_QEINT_TYPE1_PCU_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCU_VAL_PCU_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_PCU_VAL_PCU_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_PCO_MASK                                   (0x0040U)
#define CSL_EQEP_QEINT_TYPE1_PCO_SHIFT                                  (0x0006U)
#define CSL_EQEP_QEINT_TYPE1_PCO_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCO_VAL_PCO_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_PCO_VAL_PCO_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_PCR_MASK                                   (0x0080U)
#define CSL_EQEP_QEINT_TYPE1_PCR_SHIFT                                  (0x0007U)
#define CSL_EQEP_QEINT_TYPE1_PCR_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCR_VAL_PCR_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_PCR_VAL_PCR_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_PCM_MASK                                   (0x0100U)
#define CSL_EQEP_QEINT_TYPE1_PCM_SHIFT                                  (0x0008U)
#define CSL_EQEP_QEINT_TYPE1_PCM_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_PCM_VAL_PCM_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_PCM_VAL_PCM_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_SEL_MASK                                   (0x0200U)
#define CSL_EQEP_QEINT_TYPE1_SEL_SHIFT                                  (0x0009U)
#define CSL_EQEP_QEINT_TYPE1_SEL_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_SEL_VAL_SEL_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_SEL_VAL_SEL_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_IEL_MASK                                   (0x0400U)
#define CSL_EQEP_QEINT_TYPE1_IEL_SHIFT                                  (0x000AU)
#define CSL_EQEP_QEINT_TYPE1_IEL_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_IEL_VAL_IEL_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_IEL_VAL_IEL_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_UTO_MASK                                   (0x0800U)
#define CSL_EQEP_QEINT_TYPE1_UTO_SHIFT                                  (0x000BU)
#define CSL_EQEP_QEINT_TYPE1_UTO_MAX                                    (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_UTO_VAL_UTO_DISABLE                        (0x0U)
#define CSL_EQEP_QEINT_TYPE1_UTO_VAL_UTO_ENABLE                         (0x1U)

#define CSL_EQEP_QEINT_TYPE1_QMAE_MASK                                  (0x1000U)
#define CSL_EQEP_QEINT_TYPE1_QMAE_SHIFT                                 (0x000CU)
#define CSL_EQEP_QEINT_TYPE1_QMAE_MAX                                   (0x0001U)

#define CSL_EQEP_QEINT_TYPE1_QMAE_VAL_QMAE_DISABLE                      (0x0U)
#define CSL_EQEP_QEINT_TYPE1_QMAE_VAL_QMAE_ENABLE                       (0x1U)

#define CSL_EQEP_QEINT_TYPE1_RESERVED_2_MASK                            (0xE000U)
#define CSL_EQEP_QEINT_TYPE1_RESERVED_2_SHIFT                           (0x000DU)
#define CSL_EQEP_QEINT_TYPE1_RESERVED_2_MAX                             (0x0007U)

/* QFLG_TYPE1 */

#define CSL_EQEP_QFLG_TYPE1_INT_MASK                                    (0x0001U)
#define CSL_EQEP_QFLG_TYPE1_INT_SHIFT                                   (0x0000U)
#define CSL_EQEP_QFLG_TYPE1_INT_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_INT_VAL_INT_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_INT_VAL_INT_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PCE_MASK                                    (0x0002U)
#define CSL_EQEP_QFLG_TYPE1_PCE_SHIFT                                   (0x0001U)
#define CSL_EQEP_QFLG_TYPE1_PCE_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PCE_VAL_PCE_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PCE_VAL_PCE_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PHE_MASK                                    (0x0004U)
#define CSL_EQEP_QFLG_TYPE1_PHE_SHIFT                                   (0x0002U)
#define CSL_EQEP_QFLG_TYPE1_PHE_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PHE_VAL_PHE_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PHE_VAL_PHE_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_QDC_MASK                                    (0x0008U)
#define CSL_EQEP_QFLG_TYPE1_QDC_SHIFT                                   (0x0003U)
#define CSL_EQEP_QFLG_TYPE1_QDC_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_QDC_VAL_QDC_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_QDC_VAL_QDC_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_WTO_MASK                                    (0x0010U)
#define CSL_EQEP_QFLG_TYPE1_WTO_SHIFT                                   (0x0004U)
#define CSL_EQEP_QFLG_TYPE1_WTO_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_WTO_VAL_WTO_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_WTO_VAL_WTO_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PCU_MASK                                    (0x0020U)
#define CSL_EQEP_QFLG_TYPE1_PCU_SHIFT                                   (0x0005U)
#define CSL_EQEP_QFLG_TYPE1_PCU_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PCU_VAL_PCU_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PCU_VAL_PCU_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PCO_MASK                                    (0x0040U)
#define CSL_EQEP_QFLG_TYPE1_PCO_SHIFT                                   (0x0006U)
#define CSL_EQEP_QFLG_TYPE1_PCO_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PCO_VAL_PCO_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PCO_VAL_PCO_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PCR_MASK                                    (0x0080U)
#define CSL_EQEP_QFLG_TYPE1_PCR_SHIFT                                   (0x0007U)
#define CSL_EQEP_QFLG_TYPE1_PCR_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PCR_VAL_PCR_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PCR_VAL_PCR_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_PCM_MASK                                    (0x0100U)
#define CSL_EQEP_QFLG_TYPE1_PCM_SHIFT                                   (0x0008U)
#define CSL_EQEP_QFLG_TYPE1_PCM_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_PCM_VAL_PCM_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_PCM_VAL_PCM_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_SEL_MASK                                    (0x0200U)
#define CSL_EQEP_QFLG_TYPE1_SEL_SHIFT                                   (0x0009U)
#define CSL_EQEP_QFLG_TYPE1_SEL_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_SEL_VAL_SEL_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_SEL_VAL_SEL_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_IEL_MASK                                    (0x0400U)
#define CSL_EQEP_QFLG_TYPE1_IEL_SHIFT                                   (0x000AU)
#define CSL_EQEP_QFLG_TYPE1_IEL_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_IEL_VAL_IEL_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_IEL_VAL_IEL_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_UTO_MASK                                    (0x0800U)
#define CSL_EQEP_QFLG_TYPE1_UTO_SHIFT                                   (0x000BU)
#define CSL_EQEP_QFLG_TYPE1_UTO_MAX                                     (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_UTO_VAL_UTO_NOFLAG                          (0x0U)
#define CSL_EQEP_QFLG_TYPE1_UTO_VAL_UTO_FLAG                            (0x1U)

#define CSL_EQEP_QFLG_TYPE1_QMAE_MASK                                   (0x1000U)
#define CSL_EQEP_QFLG_TYPE1_QMAE_SHIFT                                  (0x000CU)
#define CSL_EQEP_QFLG_TYPE1_QMAE_MAX                                    (0x0001U)

#define CSL_EQEP_QFLG_TYPE1_QMAE_VAL_QMAE_NOFLAG                        (0x0U)
#define CSL_EQEP_QFLG_TYPE1_QMAE_VAL_QMAE_FLAG                          (0x1U)

#define CSL_EQEP_QFLG_TYPE1_RESERVED_1_MASK                             (0xE000U)
#define CSL_EQEP_QFLG_TYPE1_RESERVED_1_SHIFT                            (0x000DU)
#define CSL_EQEP_QFLG_TYPE1_RESERVED_1_MAX                              (0x0007U)

/* QCLR_TYPE1 */

#define CSL_EQEP_QCLR_TYPE1_INT_MASK                                    (0x0001U)
#define CSL_EQEP_QCLR_TYPE1_INT_SHIFT                                   (0x0000U)
#define CSL_EQEP_QCLR_TYPE1_INT_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_INT_VAL_INT_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_INT_VAL_INT_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PCE_MASK                                    (0x0002U)
#define CSL_EQEP_QCLR_TYPE1_PCE_SHIFT                                   (0x0001U)
#define CSL_EQEP_QCLR_TYPE1_PCE_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PCE_VAL_PCE_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PCE_VAL_PCE_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PHE_MASK                                    (0x0004U)
#define CSL_EQEP_QCLR_TYPE1_PHE_SHIFT                                   (0x0002U)
#define CSL_EQEP_QCLR_TYPE1_PHE_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PHE_VAL_PHE_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PHE_VAL_PHE_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_QDC_MASK                                    (0x0008U)
#define CSL_EQEP_QCLR_TYPE1_QDC_SHIFT                                   (0x0003U)
#define CSL_EQEP_QCLR_TYPE1_QDC_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_QDC_VAL_QDC_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_QDC_VAL_QDC_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_WTO_MASK                                    (0x0010U)
#define CSL_EQEP_QCLR_TYPE1_WTO_SHIFT                                   (0x0004U)
#define CSL_EQEP_QCLR_TYPE1_WTO_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_WTO_VAL_WTO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_WTO_VAL_WTO_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PCU_MASK                                    (0x0020U)
#define CSL_EQEP_QCLR_TYPE1_PCU_SHIFT                                   (0x0005U)
#define CSL_EQEP_QCLR_TYPE1_PCU_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PCU_VAL_PCU_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PCU_VAL_PCU_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PCO_MASK                                    (0x0040U)
#define CSL_EQEP_QCLR_TYPE1_PCO_SHIFT                                   (0x0006U)
#define CSL_EQEP_QCLR_TYPE1_PCO_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PCO_VAL_PCO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PCO_VAL_PCO_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PCR_MASK                                    (0x0080U)
#define CSL_EQEP_QCLR_TYPE1_PCR_SHIFT                                   (0x0007U)
#define CSL_EQEP_QCLR_TYPE1_PCR_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PCR_VAL_PCR_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PCR_VAL_PCR_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_PCM_MASK                                    (0x0100U)
#define CSL_EQEP_QCLR_TYPE1_PCM_SHIFT                                   (0x0008U)
#define CSL_EQEP_QCLR_TYPE1_PCM_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_PCM_VAL_PCM_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_PCM_VAL_PCM_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_SEL_MASK                                    (0x0200U)
#define CSL_EQEP_QCLR_TYPE1_SEL_SHIFT                                   (0x0009U)
#define CSL_EQEP_QCLR_TYPE1_SEL_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_SEL_VAL_SEL_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_SEL_VAL_SEL_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_IEL_MASK                                    (0x0400U)
#define CSL_EQEP_QCLR_TYPE1_IEL_SHIFT                                   (0x000AU)
#define CSL_EQEP_QCLR_TYPE1_IEL_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_IEL_VAL_IEL_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_IEL_VAL_IEL_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_UTO_MASK                                    (0x0800U)
#define CSL_EQEP_QCLR_TYPE1_UTO_SHIFT                                   (0x000BU)
#define CSL_EQEP_QCLR_TYPE1_UTO_MAX                                     (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_UTO_VAL_UTO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QCLR_TYPE1_UTO_VAL_UTO_CLR                             (0x1U)

#define CSL_EQEP_QCLR_TYPE1_QMAE_MASK                                   (0x1000U)
#define CSL_EQEP_QCLR_TYPE1_QMAE_SHIFT                                  (0x000CU)
#define CSL_EQEP_QCLR_TYPE1_QMAE_MAX                                    (0x0001U)

#define CSL_EQEP_QCLR_TYPE1_QMAE_VAL_QMAE_NOEFFECT                      (0x0U)
#define CSL_EQEP_QCLR_TYPE1_QMAE_VAL_QMAE_CLR                           (0x1U)

#define CSL_EQEP_QCLR_TYPE1_RESERVED_1_MASK                             (0xE000U)
#define CSL_EQEP_QCLR_TYPE1_RESERVED_1_SHIFT                            (0x000DU)
#define CSL_EQEP_QCLR_TYPE1_RESERVED_1_MAX                              (0x0007U)

/* QFRC_TYPE1 */

#define CSL_EQEP_QFRC_TYPE1_RESERVED_1_MASK                             (0x0001U)
#define CSL_EQEP_QFRC_TYPE1_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_EQEP_QFRC_TYPE1_RESERVED_1_MAX                              (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCE_MASK                                    (0x0002U)
#define CSL_EQEP_QFRC_TYPE1_PCE_SHIFT                                   (0x0001U)
#define CSL_EQEP_QFRC_TYPE1_PCE_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCE_VAL_PCE_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PCE_VAL_PCE_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_PHE_MASK                                    (0x0004U)
#define CSL_EQEP_QFRC_TYPE1_PHE_SHIFT                                   (0x0002U)
#define CSL_EQEP_QFRC_TYPE1_PHE_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PHE_VAL_PHE_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PHE_VAL_PHE_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_QDC_MASK                                    (0x0008U)
#define CSL_EQEP_QFRC_TYPE1_QDC_SHIFT                                   (0x0003U)
#define CSL_EQEP_QFRC_TYPE1_QDC_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_QDC_VAL_QDC_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_QDC_VAL_QDC_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_WTO_MASK                                    (0x0010U)
#define CSL_EQEP_QFRC_TYPE1_WTO_SHIFT                                   (0x0004U)
#define CSL_EQEP_QFRC_TYPE1_WTO_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_WTO_VAL_WTO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_WTO_VAL_WTO_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_PCU_MASK                                    (0x0020U)
#define CSL_EQEP_QFRC_TYPE1_PCU_SHIFT                                   (0x0005U)
#define CSL_EQEP_QFRC_TYPE1_PCU_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCU_VAL_PCU_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PCU_VAL_PCU_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_PCO_MASK                                    (0x0040U)
#define CSL_EQEP_QFRC_TYPE1_PCO_SHIFT                                   (0x0006U)
#define CSL_EQEP_QFRC_TYPE1_PCO_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCO_VAL_PCO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PCO_VAL_PCO_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_PCR_MASK                                    (0x0080U)
#define CSL_EQEP_QFRC_TYPE1_PCR_SHIFT                                   (0x0007U)
#define CSL_EQEP_QFRC_TYPE1_PCR_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCR_VAL_PCR_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PCR_VAL_PCR_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_PCM_MASK                                    (0x0100U)
#define CSL_EQEP_QFRC_TYPE1_PCM_SHIFT                                   (0x0008U)
#define CSL_EQEP_QFRC_TYPE1_PCM_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_PCM_VAL_PCM_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_PCM_VAL_PCM_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_SEL_MASK                                    (0x0200U)
#define CSL_EQEP_QFRC_TYPE1_SEL_SHIFT                                   (0x0009U)
#define CSL_EQEP_QFRC_TYPE1_SEL_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_SEL_VAL_SEL_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_SEL_VAL_SEL_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_IEL_MASK                                    (0x0400U)
#define CSL_EQEP_QFRC_TYPE1_IEL_SHIFT                                   (0x000AU)
#define CSL_EQEP_QFRC_TYPE1_IEL_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_IEL_VAL_IEL_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_IEL_VAL_IEL_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_UTO_MASK                                    (0x0800U)
#define CSL_EQEP_QFRC_TYPE1_UTO_SHIFT                                   (0x000BU)
#define CSL_EQEP_QFRC_TYPE1_UTO_MAX                                     (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_UTO_VAL_UTO_NOEFFECT                        (0x0U)
#define CSL_EQEP_QFRC_TYPE1_UTO_VAL_UTO_FORCE                           (0x1U)

#define CSL_EQEP_QFRC_TYPE1_QMAE_MASK                                   (0x1000U)
#define CSL_EQEP_QFRC_TYPE1_QMAE_SHIFT                                  (0x000CU)
#define CSL_EQEP_QFRC_TYPE1_QMAE_MAX                                    (0x0001U)

#define CSL_EQEP_QFRC_TYPE1_QMAE_VAL_QMAE_NOEFFECT                      (0x0U)
#define CSL_EQEP_QFRC_TYPE1_QMAE_VAL_QMAE_FORCE                         (0x1U)

#define CSL_EQEP_QFRC_TYPE1_RESERVED_2_MASK                             (0xE000U)
#define CSL_EQEP_QFRC_TYPE1_RESERVED_2_SHIFT                            (0x000DU)
#define CSL_EQEP_QFRC_TYPE1_RESERVED_2_MAX                              (0x0007U)

/* QEPSTS_TYPE1 */

#define CSL_EQEP_QEPSTS_TYPE1_PCEF_MASK                                 (0x0001U)
#define CSL_EQEP_QEPSTS_TYPE1_PCEF_SHIFT                                (0x0000U)
#define CSL_EQEP_QEPSTS_TYPE1_PCEF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_PCEF_VAL_PCEF_NOERROR                     (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_PCEF_VAL_PCEF_ERROR                       (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_FIMF_MASK                                 (0x0002U)
#define CSL_EQEP_QEPSTS_TYPE1_FIMF_SHIFT                                (0x0001U)
#define CSL_EQEP_QEPSTS_TYPE1_FIMF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_FIMF_VAL_FIMF_WRT1                        (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_FIMF_VAL_FIMF_SETINDEX                    (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_CDEF_MASK                                 (0x0004U)
#define CSL_EQEP_QEPSTS_TYPE1_CDEF_SHIFT                                (0x0002U)
#define CSL_EQEP_QEPSTS_TYPE1_CDEF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_CDEF_VAL_CDEF_WRT1                        (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_CDEF_VAL_CDEF_DIRECT                      (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_COEF_MASK                                 (0x0008U)
#define CSL_EQEP_QEPSTS_TYPE1_COEF_SHIFT                                (0x0003U)
#define CSL_EQEP_QEPSTS_TYPE1_COEF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_COEF_VAL_COEF_WRT1                        (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_COEF_VAL_COEF_OVF                         (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_QDLF_MASK                                 (0x0010U)
#define CSL_EQEP_QEPSTS_TYPE1_QDLF_SHIFT                                (0x0004U)
#define CSL_EQEP_QEPSTS_TYPE1_QDLF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_QDLF_VAL_QDLF_COUNTERCLK                  (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_QDLF_VAL_QDLF_CLK                         (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_QDF_MASK                                  (0x0020U)
#define CSL_EQEP_QEPSTS_TYPE1_QDF_SHIFT                                 (0x0005U)
#define CSL_EQEP_QEPSTS_TYPE1_QDF_MAX                                   (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_QDF_VAL_QDF_COUNTERCLK                    (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_QDF_VAL_QDF_CLK                           (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_FIDF_MASK                                 (0x0040U)
#define CSL_EQEP_QEPSTS_TYPE1_FIDF_SHIFT                                (0x0006U)
#define CSL_EQEP_QEPSTS_TYPE1_FIDF_MAX                                  (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_FIDF_VAL_FIDF_COUNTERCLK                  (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_FIDF_VAL_FIDF_CLK                         (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_UPEVNT_MASK                               (0x0080U)
#define CSL_EQEP_QEPSTS_TYPE1_UPEVNT_SHIFT                              (0x0007U)
#define CSL_EQEP_QEPSTS_TYPE1_UPEVNT_MAX                                (0x0001U)

#define CSL_EQEP_QEPSTS_TYPE1_UPEVNT_VAL_UPEVNT_NODETCT                 (0x0U)
#define CSL_EQEP_QEPSTS_TYPE1_UPEVNT_VAL_UPEVNT_DETCT                   (0x1U)

#define CSL_EQEP_QEPSTS_TYPE1_RESERVED_1_MASK                           (0xFF00U)
#define CSL_EQEP_QEPSTS_TYPE1_RESERVED_1_SHIFT                          (0x0008U)
#define CSL_EQEP_QEPSTS_TYPE1_RESERVED_1_MAX                            (0x00FFU)

/* QCTMR */

#define CSL_EQEP_QCTMR_QCTMR_MASK                                       (0xFFFFU)
#define CSL_EQEP_QCTMR_QCTMR_SHIFT                                      (0x0000U)
#define CSL_EQEP_QCTMR_QCTMR_MAX                                        (0xFFFFU)

/* QCPRD */

#define CSL_EQEP_QCPRD_QCPRD_MASK                                       (0xFFFFU)
#define CSL_EQEP_QCPRD_QCPRD_SHIFT                                      (0x0000U)
#define CSL_EQEP_QCPRD_QCPRD_MAX                                        (0xFFFFU)

/* QCTMRLAT */

#define CSL_EQEP_QCTMRLAT_QCTMRLAT_MASK                                 (0xFFFFU)
#define CSL_EQEP_QCTMRLAT_QCTMRLAT_SHIFT                                (0x0000U)
#define CSL_EQEP_QCTMRLAT_QCTMRLAT_MAX                                  (0xFFFFU)

/* QCPRDLAT */

#define CSL_EQEP_QCPRDLAT_QCPRDLAT_MASK                                 (0xFFFFU)
#define CSL_EQEP_QCPRDLAT_QCPRDLAT_SHIFT                                (0x0000U)
#define CSL_EQEP_QCPRDLAT_QCPRDLAT_MAX                                  (0xFFFFU)

/* RESERVED_1 */

/* PID */

#define CSL_EQEP_PID_MINOR_MASK                                         (0x0000003FU)
#define CSL_EQEP_PID_MINOR_SHIFT                                        (0x00000000U)
#define CSL_EQEP_PID_MINOR_MAX                                          (0x0000003FU)

#define CSL_EQEP_PID_CUSTOM_MASK                                        (0x000000C0U)
#define CSL_EQEP_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define CSL_EQEP_PID_CUSTOM_MAX                                         (0x00000003U)

#define CSL_EQEP_PID_MAJOR_MASK                                         (0x00000700U)
#define CSL_EQEP_PID_MAJOR_SHIFT                                        (0x00000008U)
#define CSL_EQEP_PID_MAJOR_MAX                                          (0x00000007U)

#define CSL_EQEP_PID_RTL_MASK                                           (0x0000F800U)
#define CSL_EQEP_PID_RTL_SHIFT                                          (0x0000000BU)
#define CSL_EQEP_PID_RTL_MAX                                            (0x0000001FU)

#define CSL_EQEP_PID_FUNCTION_MASK                                      (0x0FFF0000U)
#define CSL_EQEP_PID_FUNCTION_SHIFT                                     (0x00000010U)
#define CSL_EQEP_PID_FUNCTION_MAX                                       (0x00000FFFU)

#define CSL_EQEP_PID_RESERVED_MASK                                      (0x30000000U)
#define CSL_EQEP_PID_RESERVED_SHIFT                                     (0x0000001CU)
#define CSL_EQEP_PID_RESERVED_MAX                                       (0x00000003U)

#define CSL_EQEP_PID_SCHEME_MASK                                        (0xC0000000U)
#define CSL_EQEP_PID_SCHEME_SHIFT                                       (0x0000001EU)
#define CSL_EQEP_PID_SCHEME_MAX                                         (0x00000003U)

/* REV_TYPE2 */

#define CSL_EQEP_REV_TYPE2_MAJOR_MASK                                   (0x00000007U)
#define CSL_EQEP_REV_TYPE2_MAJOR_SHIFT                                  (0x00000000U)
#define CSL_EQEP_REV_TYPE2_MAJOR_MAX                                    (0x00000007U)

#define CSL_EQEP_REV_TYPE2_MINOR_MASK                                   (0x00000038U)
#define CSL_EQEP_REV_TYPE2_MINOR_SHIFT                                  (0x00000003U)
#define CSL_EQEP_REV_TYPE2_MINOR_MAX                                    (0x00000007U)

#define CSL_EQEP_REV_TYPE2_RESERVED_1_MASK                              (0xFFFFFFC0U)
#define CSL_EQEP_REV_TYPE2_RESERVED_1_SHIFT                             (0x00000006U)
#define CSL_EQEP_REV_TYPE2_RESERVED_1_MAX                               (0x03FFFFFFU)

/* QEPSTROBESEL */

#define CSL_EQEP_QEPSTROBESEL_STROBESEL_MASK                            (0x00000003U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_SHIFT                           (0x00000000U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_MAX                             (0x00000003U)

#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_QS_AFTER_POL_MUX            (0x0U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_QS_AFTER_POL_MUX_1          (0x1U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_ADCSOCA_AS_QS               (0x2U)
#define CSL_EQEP_QEPSTROBESEL_STROBESEL_VAL_ADCSOCB_AS_QS               (0x3U)

#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_MASK                           (0xFFFFFFFCU)
#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_SHIFT                          (0x00000002U)
#define CSL_EQEP_QEPSTROBESEL_RESERVED_1_MAX                            (0x3FFFFFFFU)

/* QMACTRL */

#define CSL_EQEP_QMACTRL_MODE_MASK                                      (0x00000007U)
#define CSL_EQEP_QMACTRL_MODE_SHIFT                                     (0x00000000U)
#define CSL_EQEP_QMACTRL_MODE_MAX                                       (0x00000007U)

#define CSL_EQEP_QMACTRL_RESERVED_1_MASK                                (0xFFFFFFF8U)
#define CSL_EQEP_QMACTRL_RESERVED_1_SHIFT                               (0x00000003U)
#define CSL_EQEP_QMACTRL_RESERVED_1_MAX                                 (0x1FFFFFFFU)

/* QEPSRCSEL */

#define CSL_EQEP_QEPSRCSEL_QEPASEL_MASK                                 (0x0000000FU)
#define CSL_EQEP_QEPSRCSEL_QEPASEL_SHIFT                                (0x00000000U)
#define CSL_EQEP_QEPSRCSEL_QEPASEL_MAX                                  (0x0000000FU)

#define CSL_EQEP_QEPSRCSEL_QEPBSEL_MASK                                 (0x000000F0U)
#define CSL_EQEP_QEPSRCSEL_QEPBSEL_SHIFT                                (0x00000004U)
#define CSL_EQEP_QEPSRCSEL_QEPBSEL_MAX                                  (0x0000000FU)

#define CSL_EQEP_QEPSRCSEL_QEPISEL_MASK                                 (0x00000F00U)
#define CSL_EQEP_QEPSRCSEL_QEPISEL_SHIFT                                (0x00000008U)
#define CSL_EQEP_QEPSRCSEL_QEPISEL_MAX                                  (0x0000000FU)

#define CSL_EQEP_QEPSRCSEL_QEPSSEL_MASK                                 (0x0000F000U)
#define CSL_EQEP_QEPSRCSEL_QEPSSEL_SHIFT                                (0x0000000CU)
#define CSL_EQEP_QEPSRCSEL_QEPSSEL_MAX                                  (0x0000000FU)

#define CSL_EQEP_QEPSRCSEL_RESERVED_1_MASK                              (0xFFFF0000U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_1_SHIFT                             (0x00000010U)
#define CSL_EQEP_QEPSRCSEL_RESERVED_1_MAX                               (0x0000FFFFU)

/* RESERVED_2 */

#ifdef __cplusplus
}
#endif
#endif
