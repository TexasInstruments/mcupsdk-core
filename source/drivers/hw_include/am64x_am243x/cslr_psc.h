/********************************************************************
 * Copyright (C) 2018 Texas Instruments Incorporated.
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
 *  Name        : cslr_psc.h
*/
#ifndef CSLR_PSC_H_
#define CSLR_PSC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/


/**************************************************************************
* Hardware Region  : MMRs in region 0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* PID register */
    volatile uint8_t  Resv_20[16];
    volatile uint32_t GBLSTAT;                   /* Global Status Register */
    volatile uint32_t INTEVAL;                   /* Interrupt Evaluation Register */
    volatile uint32_t IPWKCNT;                   /* IP WakeUp Count Register (should be treated as read-only) */
    volatile uint8_t  Resv_64[32];
    volatile uint32_t MERRPR[4];                 /* Module Error Pending Register */
    volatile uint32_t MERRCR[4];                 /* Module Error Clear Register */
    volatile uint32_t PERRPR[2];                 /* Power Domain Error Pending (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint32_t PERRCR[2];                 /* Power Domain Error Clear (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint32_t EPCPR[2];                  /* External Power Control Pending (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint32_t EPCCR[2];                  /* External Power Control Clear (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint8_t  Resv_256[128];
    volatile uint32_t RAILSTAT;                  /* Power Rail Status Register */
    volatile uint32_t RAILCTL;                   /* Power Rail Counter Control Register */
    volatile uint32_t RAILSEL[2];                /* Power Rail Counter Select (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint8_t  Resv_288[16];
    volatile uint32_t PTCMD[2];                  /* Power Domain Transition Command (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint32_t PTSTAT[2];                 /* Power Domain Transition Status (index 0: PD 0 to 31, index 1: PD 32 to 63) */
    volatile uint8_t  Resv_512[208];
    volatile uint32_t PDSTAT[64];                /* Power Domain Status */
    volatile uint32_t PDCTL[64];                 /* Power Domain Control */
    volatile uint32_t PDCFG[64];                 /* Power Domain Configuration */
    volatile uint8_t  Resv_1536[256];
    volatile uint32_t MDCFG[128];                /* Module Configuration */
    volatile uint32_t MDSTAT[128];               /* Module Status */
    volatile uint32_t MDCTL[128];                /* Module Control */
} CSL_PscRegs;


/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_PscRegs             *CSL_PscRegsOvly;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PSC_PID                                          (0x00000000U)
#define CSL_PSC_GBLSTAT                                      (0x00000014U)
#define CSL_PSC_INTEVAL                                      (0x00000018U)
#define CSL_PSC_IPWKCNT                                      (0x0000001CU)
#define CSL_PSC_MERRPR(n)                                    (0x00000040U+((n)*0x0004U))
#define CSL_PSC_MERRCR(n)                                    (0x00000050U+((n)*0x0004U))
#define CSL_PSC_PERRPR(n)                                    (0x00000060U+((n)*0x0004U))
#define CSL_PSC_PERRCR(n)                                    (0x00000068U+((n)*0x0004U))
#define CSL_PSC_EPCPR(n)                                     (0x00000070U+((n)*0x0004U))
#define CSL_PSC_EPCCR(n)                                     (0x00000078U+((n)*0x0004U))
#define CSL_PSC_RAILSTAT                                     (0x00000100U)
#define CSL_PSC_RAILCTL                                      (0x00000104U)
#define CSL_PSC_RAILSEL(n)                                   (0x00000108U+((n)*0x0004U))
#define CSL_PSC_PTCMD(n)                                     (0x00000120U+((n)*0x0004U))
#define CSL_PSC_PTSTAT(n)                                    (0x00000128U+((n)*0x0004U))
#define CSL_PSC_PDSTAT(n)                                    (0x00000200U+((n)*0x0004U))
#define CSL_PSC_PDCTL(n)                                     (0x00000300U+((n)*0x0004U))
#define CSL_PSC_PDCFG(n)                                     (0x00000400U+((n)*0x0004U))
#define CSL_PSC_MDCFG(n)                                     (0x00000600U+((n)*0x0004U))
#define CSL_PSC_MDSTAT(n)                                    (0x00000800U+((n)*0x0004U))
#define CSL_PSC_MDCTL(n)                                     (0x00000A00U+((n)*0x0004U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_PSC_PID_MINOR_MASK                               (0x0000003FU)
#define CSL_PSC_PID_MINOR_SHIFT                              (0x00000000U)
#define CSL_PSC_PID_MINOR_RESETVAL                           (0x00000000U)
#define CSL_PSC_PID_MINOR_MAX                                (0x0000003FU)

#define CSL_PSC_PID_CUSTOM_MASK                              (0x000000C0U)
#define CSL_PSC_PID_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_PSC_PID_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_PSC_PID_CUSTOM_MAX                               (0x00000003U)

#define CSL_PSC_PID_MAJOR_MASK                               (0x00000700U)
#define CSL_PSC_PID_MAJOR_SHIFT                              (0x00000008U)
#define CSL_PSC_PID_MAJOR_RESETVAL                           (0x00000002U)
#define CSL_PSC_PID_MAJOR_MAX                                (0x00000007U)

#define CSL_PSC_PID_RTL_MASK                                 (0x0000F800U)
#define CSL_PSC_PID_RTL_SHIFT                                (0x0000000BU)
#define CSL_PSC_PID_RTL_RESETVAL                             (0x00000000U)
#define CSL_PSC_PID_RTL_MAX                                  (0x0000001FU)

#define CSL_PSC_PID_FUNC_MASK                                (0x0FFF0000U)
#define CSL_PSC_PID_FUNC_SHIFT                               (0x00000010U)
#define CSL_PSC_PID_FUNC_RESETVAL                            (0x00000482U)
#define CSL_PSC_PID_FUNC_MAX                                 (0x00000FFFU)

#define CSL_PSC_PID_BU_MASK                                  (0x30000000U)
#define CSL_PSC_PID_BU_SHIFT                                 (0x0000001CU)
#define CSL_PSC_PID_BU_RESETVAL                              (0x00000000U)
#define CSL_PSC_PID_BU_MAX                                   (0x00000003U)

#define CSL_PSC_PID_SCHEME_MASK                              (0xC0000000U)
#define CSL_PSC_PID_SCHEME_SHIFT                             (0x0000001EU)
#define CSL_PSC_PID_SCHEME_RESETVAL                          (0x00000001U)
#define CSL_PSC_PID_SCHEME_MAX                               (0x00000003U)

#define CSL_PSC_PID_RESETVAL                                 (0x44820200U)

/* GBLSTAT */

#define CSL_PSC_GBLSTAT_OVRIDE_MASK                          (0x00000001U)
#define CSL_PSC_GBLSTAT_OVRIDE_SHIFT                         (0x00000000U)
#define CSL_PSC_GBLSTAT_OVRIDE_RESETVAL                      (0x00000000U)
#define CSL_PSC_GBLSTAT_OVRIDE_MAX                           (0x00000001U)

#define CSL_PSC_GBLSTAT_RESETVAL                             (0x00000000U)

/* INTEVAL */

#define CSL_PSC_INTEVAL_GOSET_MASK                           (0x00080000U)
#define CSL_PSC_INTEVAL_GOSET_SHIFT                          (0x00000013U)
#define CSL_PSC_INTEVAL_GOSET_RESETVAL                       (0x00000000U)
#define CSL_PSC_INTEVAL_GOSET_MAX                            (0x00000001U)

#define CSL_PSC_INTEVAL_EPCSET_MASK                          (0x00040000U)
#define CSL_PSC_INTEVAL_EPCSET_SHIFT                         (0x00000012U)
#define CSL_PSC_INTEVAL_EPCSET_RESETVAL                      (0x00000000U)
#define CSL_PSC_INTEVAL_EPCSET_MAX                           (0x00000001U)

#define CSL_PSC_INTEVAL_ERRSET_MASK                          (0x00020000U)
#define CSL_PSC_INTEVAL_ERRSET_SHIFT                         (0x00000011U)
#define CSL_PSC_INTEVAL_ERRSET_RESETVAL                      (0x00000000U)
#define CSL_PSC_INTEVAL_ERRSET_MAX                           (0x00000001U)

#define CSL_PSC_INTEVAL_ALLSET_MASK                          (0x00010000U)
#define CSL_PSC_INTEVAL_ALLSET_SHIFT                         (0x00000010U)
#define CSL_PSC_INTEVAL_ALLSET_RESETVAL                      (0x00000000U)
#define CSL_PSC_INTEVAL_ALLSET_MAX                           (0x00000001U)

#define CSL_PSC_INTEVAL_EPCEV_MASK                           (0x00000004U)
#define CSL_PSC_INTEVAL_EPCEV_SHIFT                          (0x00000002U)
#define CSL_PSC_INTEVAL_EPCEV_RESETVAL                       (0x00000000U)
#define CSL_PSC_INTEVAL_EPCEV_MAX                            (0x00000001U)

#define CSL_PSC_INTEVAL_ERREV_MASK                           (0x00000002U)
#define CSL_PSC_INTEVAL_ERREV_SHIFT                          (0x00000001U)
#define CSL_PSC_INTEVAL_ERREV_RESETVAL                       (0x00000000U)
#define CSL_PSC_INTEVAL_ERREV_MAX                            (0x00000001U)

#define CSL_PSC_INTEVAL_ALLEV_MASK                           (0x00000001U)
#define CSL_PSC_INTEVAL_ALLEV_SHIFT                          (0x00000000U)
#define CSL_PSC_INTEVAL_ALLEV_RESETVAL                       (0x00000000U)
#define CSL_PSC_INTEVAL_ALLEV_MAX                            (0x00000001U)
/*----ALLEV Tokens----*/
#define CSL_PSC_INTEVAL_ALLEV_NO_EFFECT         (0x00000000u)
#define CSL_PSC_INTEVAL_ALLEV_RE_EVALUATE       (0x00000001u)

#define CSL_PSC_INTEVAL_RESETVAL                             (0x00000000U)

/* MERRPR */

#define CSL_PSC_MERRPR_M_MASK                                (0xFFFFFFFFU)
#define CSL_PSC_MERRPR_M_SHIFT                               (0x00000000U)
#define CSL_PSC_MERRPR_M_RESETVAL                            (0x00000000U)
#define CSL_PSC_MERRPR_M_MAX                                 (0xFFFFFFFFU)

#define CSL_PSC_MERRPR_RESETVAL                              (0x00000000U)

/* MERRCR */

#define CSL_PSC_MERRCR_M_MASK                                (0xFFFFFFFFU)
#define CSL_PSC_MERRCR_M_SHIFT                               (0x00000000U)
#define CSL_PSC_MERRCR_M_RESETVAL                            (0x00000000U)
#define CSL_PSC_MERRCR_M_MAX                                 (0xFFFFFFFFU)

#define CSL_PSC_MERRCR_RESETVAL                              (0x00000000U)

/* PERRPR */

#define CSL_PSC_PERRPR_P_MASK                                (0xFFFFFFFFU)
#define CSL_PSC_PERRPR_P_SHIFT                               (0x00000000U)
#define CSL_PSC_PERRPR_P_RESETVAL                            (0x00000000U)
#define CSL_PSC_PERRPR_P_MAX                                 (0xFFFFFFFFU)

#define CSL_PSC_PERRPR_RESETVAL                              (0x00000000U)

/* PERRCR */

#define CSL_PSC_PERRCR_P_MASK                                (0xFFFFFFFFU)
#define CSL_PSC_PERRCR_P_SHIFT                               (0x00000000U)
#define CSL_PSC_PERRCR_P_RESETVAL                            (0x00000000U)
#define CSL_PSC_PERRCR_P_MAX                                 (0xFFFFFFFFU)

#define CSL_PSC_PERRCR_RESETVAL                              (0x00000000U)

/* EPCPR */

#define CSL_PSC_EPCPR_EPC_MASK                               (0xFFFFFFFFU)
#define CSL_PSC_EPCPR_EPC_SHIFT                              (0x00000000U)
#define CSL_PSC_EPCPR_EPC_RESETVAL                           (0x00000000U)
#define CSL_PSC_EPCPR_EPC_MAX                                (0xFFFFFFFFU)

#define CSL_PSC_EPCPR_RESETVAL                               (0x00000000U)

/* EPCCR */

#define CSL_PSC_EPCCR_EPC_MASK                               (0xFFFFFFFFU)
#define CSL_PSC_EPCCR_EPC_SHIFT                              (0x00000000U)
#define CSL_PSC_EPCCR_EPC_RESETVAL                           (0x00000000U)
#define CSL_PSC_EPCCR_EPC_MAX                                (0xFFFFFFFFU)

#define CSL_PSC_EPCCR_RESETVAL                               (0x00000000U)

/* RAILSTAT */

#define CSL_PSC_RAILSTAT_RAILNUM_MASK                        (0x1F000000U)
#define CSL_PSC_RAILSTAT_RAILNUM_SHIFT                       (0x00000018U)
#define CSL_PSC_RAILSTAT_RAILNUM_RESETVAL                    (0x00000000U)
#define CSL_PSC_RAILSTAT_RAILNUM_MAX                         (0x0000001FU)

#define CSL_PSC_RAILSTAT_RAILCNT_MASK                        (0x000000FFU)
#define CSL_PSC_RAILSTAT_RAILCNT_SHIFT                       (0x00000000U)
#define CSL_PSC_RAILSTAT_RAILCNT_RESETVAL                    (0x00000000U)
#define CSL_PSC_RAILSTAT_RAILCNT_MAX                         (0x000000FFU)

#define CSL_PSC_RAILSTAT_RESETVAL                            (0x00000000U)

/* RAILCTL */

#define CSL_PSC_RAILCTL_RAILCTR1_MASK                        (0x0000FF00U)
#define CSL_PSC_RAILCTL_RAILCTR1_SHIFT                       (0x00000008U)
#define CSL_PSC_RAILCTL_RAILCTR1_RESETVAL                    (0x00000000U)
#define CSL_PSC_RAILCTL_RAILCTR1_MAX                         (0x000000FFU)

#define CSL_PSC_RAILCTL_RAILCTR0_MASK                        (0x000000FFU)
#define CSL_PSC_RAILCTL_RAILCTR0_SHIFT                       (0x00000000U)
#define CSL_PSC_RAILCTL_RAILCTR0_RESETVAL                    (0x00000000U)
#define CSL_PSC_RAILCTL_RAILCTR0_MAX                         (0x000000FFU)

#define CSL_PSC_RAILCTL_RESETVAL                             (0x00000000U)

/* RAILSEL */

#define CSL_PSC_RAILSEL_P_MASK                               (0xFFFFFFFFU)
#define CSL_PSC_RAILSEL_P_SHIFT                              (0x00000000U)
#define CSL_PSC_RAILSEL_P_RESETVAL                           (0x00000000U)
#define CSL_PSC_RAILSEL_P_MAX                                (0xFFFFFFFFU)

#define CSL_PSC_RAILSEL_RESETVAL                             (0x00000000U)

/* PTCMD */

#define CSL_PSC_PTCMD_GO_MASK                                (0xFFFFFFFFU)
#define CSL_PSC_PTCMD_GO_SHIFT                               (0x00000000U)
#define CSL_PSC_PTCMD_GO_RESETVAL                            (0x00000000U)
#define CSL_PSC_PTCMD_GO_MAX                                 (0xFFFFFFFFU)
/*----GO Tokens----*/
#define CSL_PSC_PTCMD_GO_SET                    (0x00000001u)

#define CSL_PSC_PTCMD_RESETVAL                               (0x00000000U)

/* PTSTAT */

#define CSL_PSC_PTSTAT_GOSTAT_MASK                           (0xFFFFFFFFU)
#define CSL_PSC_PTSTAT_GOSTAT_SHIFT                          (0x00000000U)
#define CSL_PSC_PTSTAT_GOSTAT_RESETVAL                       (0x00000000U)
#define CSL_PSC_PTSTAT_GOSTAT_MAX                            (0xFFFFFFFFU)
/*----GOSTAT Tokens----*/
#define CSL_PSC_PTSTAT_GOSTAT_NO_PWRDMN_TRANS   (0x00000000u)
#define CSL_PSC_PTSTAT_GOSTAT_PWRDMN_TRANS_PROG (0x00000001u)

#define CSL_PSC_PTSTAT_RESETVAL                              (0x00000000U)

/* PDSTAT */

#define CSL_PSC_PDSTAT_EMUIHB_MASK                          (0x00000800U)
#define CSL_PSC_PDSTAT_EMUIHB_SHIFT                         (0x0000000BU)
#define CSL_PSC_PDSTAT_EMUIHB_RESETVAL                      (0x00000000U)
#define CSL_PSC_PDSTAT_EMUIHB_MAX                           (0x00000001U)

#define CSL_PSC_PDSTAT_PWRBAD_MASK                          (0x00000400U)
#define CSL_PSC_PDSTAT_PWRBAD_SHIFT                         (0x0000000AU)
#define CSL_PSC_PDSTAT_PWRBAD_RESETVAL                      (0x00000000U)
#define CSL_PSC_PDSTAT_PWRBAD_MAX                           (0x00000001U)

#define CSL_PSC_PDSTAT_PORDONE_MASK                         (0x00000200U)
#define CSL_PSC_PDSTAT_PORDONE_SHIFT                        (0x00000009U)
#define CSL_PSC_PDSTAT_PORDONE_RESETVAL                     (0x00000000U)
#define CSL_PSC_PDSTAT_PORDONE_MAX                          (0x00000001U)

#define CSL_PSC_PDSTAT_PORZ_MASK                            (0x00000100U)
#define CSL_PSC_PDSTAT_PORZ_SHIFT                           (0x00000008U)
#define CSL_PSC_PDSTAT_PORZ_RESETVAL                        (0x00000000U)
#define CSL_PSC_PDSTAT_PORZ_MAX                             (0x00000001U)

#define CSL_PSC_PDSTAT_STATE_MASK                           (0x0000001FU)
#define CSL_PSC_PDSTAT_STATE_SHIFT                          (0x00000000U)
#define CSL_PSC_PDSTAT_STATE_RESETVAL                       (0x00000000U)
#define CSL_PSC_PDSTAT_STATE_MAX                            (0x0000001FU)
/*----STATE Tokens----*/
#define CSL_PSC_PDSTAT_STATE_OFF                (0x00000000U)
#define CSL_PSC_PDSTAT_STATE_ON                 (0x00000001U)

#define CSL_PSC_PDSTAT_RESETVAL                             (0x00000000U)

/* PDCTL */

#define CSL_PSC_PDCTL_FORCE_MASK                            (0x80000000U)
#define CSL_PSC_PDCTL_FORCE_SHIFT                           (0x0000001FU)
#define CSL_PSC_PDCTL_FORCE_RESETVAL                        (0x00000000U)
#define CSL_PSC_PDCTL_FORCE_MAX                             (0x00000001U)

#define CSL_PSC_PDCTL_PWRSW_MASK                            (0x20000000U)
#define CSL_PSC_PDCTL_PWRSW_SHIFT                           (0x0000001DU)
#define CSL_PSC_PDCTL_PWRSW_RESETVAL                        (0x00000000U)
#define CSL_PSC_PDCTL_PWRSW_MAX                             (0x00000001U)

#define CSL_PSC_PDCTL_ISO_MASK                              (0x10000000U)
#define CSL_PSC_PDCTL_ISO_SHIFT                             (0x0000001CU)
#define CSL_PSC_PDCTL_ISO_RESETVAL                          (0x00000000U)
#define CSL_PSC_PDCTL_ISO_MAX                               (0x00000001U)

#define CSL_PSC_PDCTL_WAKECNT_MASK                          (0x00FF0000U)
#define CSL_PSC_PDCTL_WAKECNT_SHIFT                         (0x00000010U)
#define CSL_PSC_PDCTL_WAKECNT_RESETVAL                      (0x00000000U)
#define CSL_PSC_PDCTL_WAKECNT_MAX                           (0x000000FFU)

#define CSL_PSC_PDCTL_PDMODE_MASK                           (0x00007000U)
#define CSL_PSC_PDCTL_PDMODE_SHIFT                          (0x0000000CU)
#define CSL_PSC_PDCTL_PDMODE_RESETVAL                       (0x00000000U)
#define CSL_PSC_PDCTL_PDMODE_MAX                            (0x00000007U)
/*----PDMODE Tokens----*/
#define CSL_PSC_PDCTL_PDMODE_OFF                (0x00000000u)
#define CSL_PSC_PDCTL_PDMODE_RAM_OFF            (0x00000008u)
#define CSL_PSC_PDCTL_PDMODE_DEEP_SLEEP         (0x00000009u)
#define CSL_PSC_PDCTL_PDMODE_LIGHT_SLEEP        (0x0000000Au)
#define CSL_PSC_PDCTL_PDMODE_RETENTION          (0x0000000Bu)
#define CSL_PSC_PDCTL_PDMODE_ON                 (0x0000000Fu)

#define CSL_PSC_PDCTL_EMUIHBIE_MASK                         (0x00000200U)
#define CSL_PSC_PDCTL_EMUIHBIE_SHIFT                        (0x00000009U)
#define CSL_PSC_PDCTL_EMUIHBIE_RESETVAL                     (0x00000000U)
#define CSL_PSC_PDCTL_EMUIHBIE_MAX                          (0x00000001U)
/*----EMUIHBIE Tokens----*/
#define CSL_PSC_PDCTL_EMUIHBIE_DISABLE          (0x00000000u)
#define CSL_PSC_PDCTL_EMUIHBIE_ENABLE           (0x00000001u)

#define CSL_PSC_PDCTL_EPCGOOD_MASK                          (0x00000100U)
#define CSL_PSC_PDCTL_EPCGOOD_SHIFT                         (0x00000008U)
#define CSL_PSC_PDCTL_EPCGOOD_RESETVAL                      (0x00000000U)
#define CSL_PSC_PDCTL_EPCGOOD_MAX                           (0x00000001U)

#define CSL_PSC_PDCTL_NEXT_MASK                             (0x00000001U)
#define CSL_PSC_PDCTL_NEXT_SHIFT                            (0x00000000U)
#define CSL_PSC_PDCTL_NEXT_RESETVAL                         (0x00000001U)
#define CSL_PSC_PDCTL_NEXT_MAX                              (0x00000001U)
/*----NEXT Tokens----*/
#define CSL_PSC_PDCTL_NEXT_OFF                  (0x00000000u)
#define CSL_PSC_PDCTL_NEXT_ON                   (0x00000001u)

#define CSL_PSC_PDCTL_RESETVAL                              (0x00000001U)

/* PDCFG */

#define CSL_PSC_PDCFG_NEXTLOCK_MASK                         (0x00000008U)
#define CSL_PSC_PDCFG_NEXTLOCK_SHIFT                        (0x00000003U)
#define CSL_PSC_PDCFG_NEXTLOCK_RESETVAL                     (0x00000000U)
#define CSL_PSC_PDCFG_NEXTLOCK_MAX                          (0x00000001U)

#define CSL_PSC_PDCFG_ICEPICK_MASK                          (0x00000004U)
#define CSL_PSC_PDCFG_ICEPICK_SHIFT                         (0x00000002U)
#define CSL_PSC_PDCFG_ICEPICK_RESETVAL                      (0x00000000U)
#define CSL_PSC_PDCFG_ICEPICK_MAX                           (0x00000001U)
/*----ICEPICK Tokens----*/
#define CSL_PSC_PDCFG_ICEPICK_ABSENT            (0x00000000u)
#define CSL_PSC_PDCFG_ICEPICK_PRESENT           (0x00000001u)

#define CSL_PSC_PDCFG_MEMSLPKWK_MASK                        (0x00000002U)
#define CSL_PSC_PDCFG_MEMSLPKWK_SHIFT                       (0x00000001U)
#define CSL_PSC_PDCFG_MEMSLPKWK_RESETVAL                    (0x00000000U)
#define CSL_PSC_PDCFG_MEMSLPKWK_MAX                         (0x00000001U)

#define CSL_PSC_PDCFG_ALWAYSON_MASK                         (0x00000001U)
#define CSL_PSC_PDCFG_ALWAYSON_SHIFT                        (0x00000000U)
#define CSL_PSC_PDCFG_ALWAYSON_RESETVAL                     (0x00000000U)
#define CSL_PSC_PDCFG_ALWAYSON_MAX                          (0x00000001U)
/*----ALWAYSON Tokens----*/
#define CSL_PSC_PDCFG_ALWAYSON_NO               (0x00000000u)
#define CSL_PSC_PDCFG_ALWAYSON_YES              (0x00000001u)

#define CSL_PSC_PDCFG_RESETVAL                              (0x00000000U)

/* MDCFG */

#define CSL_PSC_MDCFG_PWRDOM_MASK                           (0x001F0000U)
#define CSL_PSC_MDCFG_PWRDOM_SHIFT                          (0x00000010U)
#define CSL_PSC_MDCFG_PWRDOM_RESETVAL                       (0x00000000U)
#define CSL_PSC_MDCFG_PWRDOM_MAX                            (0x0000001FU)

#define CSL_PSC_MDCFG_AUTOONLY_MASK                         (0x00008000U)
#define CSL_PSC_MDCFG_AUTOONLY_SHIFT                        (0x0000000FU)
#define CSL_PSC_MDCFG_AUTOONLY_RESETVAL                     (0x00000000U)
#define CSL_PSC_MDCFG_AUTOONLY_MAX                          (0x00000001U)

#define CSL_PSC_MDCFG_RESETISO_MASK                         (0x00004000U)
#define CSL_PSC_MDCFG_RESETISO_SHIFT                        (0x0000000EU)
#define CSL_PSC_MDCFG_RESETISO_RESETVAL                     (0x00000000U)
#define CSL_PSC_MDCFG_RESETISO_MAX                          (0x00000001U)

#define CSL_PSC_MDCFG_NEXTLOCK_MASK                         (0x00002000U)
#define CSL_PSC_MDCFG_NEXTLOCK_SHIFT                        (0x0000000DU)
#define CSL_PSC_MDCFG_NEXTLOCK_RESETVAL                     (0x00000000U)
#define CSL_PSC_MDCFG_NEXTLOCK_MAX                          (0x00000001U)

#define CSL_PSC_MDCFG_ASYNC_MASK                            (0x00001000U)
#define CSL_PSC_MDCFG_ASYNC_SHIFT                           (0x0000000CU)
#define CSL_PSC_MDCFG_ASYNC_RESETVAL                        (0x00000000U)
#define CSL_PSC_MDCFG_ASYNC_MAX                             (0x00000001U)

#define CSL_PSC_MDCFG_ICEPICK_MASK                          (0x00000800U)
#define CSL_PSC_MDCFG_ICEPICK_SHIFT                         (0x0000000BU)
#define CSL_PSC_MDCFG_ICEPICK_RESETVAL                      (0x00000000U)
#define CSL_PSC_MDCFG_ICEPICK_MAX                           (0x00000001U)

#define CSL_PSC_MDCFG_PERMDIS_MASK                          (0x00000400U)
#define CSL_PSC_MDCFG_PERMDIS_SHIFT                         (0x0000000AU)
#define CSL_PSC_MDCFG_PERMDIS_RESETVAL                      (0x00000000U)
#define CSL_PSC_MDCFG_PERMDIS_MAX                           (0x00000001U)

#define CSL_PSC_MDCFG_PLLHANDSHAKE_MASK                     (0x00000200U)
#define CSL_PSC_MDCFG_PLLHANDSHAKE_SHIFT                    (0x00000009U)
#define CSL_PSC_MDCFG_PLLHANDSHAKE_RESETVAL                 (0x00000000U)
#define CSL_PSC_MDCFG_PLLHANDSHAKE_MAX                      (0x00000001U)

#define CSL_PSC_MDCFG_NUMSCRDISBALE_MASK                    (0x000001C0U)
#define CSL_PSC_MDCFG_NUMSCRDISBALE_SHIFT                   (0x00000006U)
#define CSL_PSC_MDCFG_NUMSCRDISBALE_RESETVAL                (0x00000000U)
#define CSL_PSC_MDCFG_NUMSCRDISBALE_MAX                     (0x00000007U)

#define CSL_PSC_MDCFG_NUMCLKEN_MASK                         (0x00000038U)
#define CSL_PSC_MDCFG_NUMCLKEN_SHIFT                        (0x00000003U)
#define CSL_PSC_MDCFG_NUMCLKEN_RESETVAL                     (0x00000000U)
#define CSL_PSC_MDCFG_NUMCLKEN_MAX                          (0x00000007U)

#define CSL_PSC_MDCFG_NUMCLK_MASK                           (0x00000007U)
#define CSL_PSC_MDCFG_NUMCLK_SHIFT                          (0x00000000U)
#define CSL_PSC_MDCFG_NUMCLK_RESETVAL                       (0x00000000U)
#define CSL_PSC_MDCFG_NUMCLK_MAX                            (0x00000007U)

#define CSL_PSC_MDCFG_RESETVAL                              (0x00000000U)

/* MDSTAT */

#define CSL_PSC_MDSTAT_EMUIHB_MASK                          (0x00020000U)
#define CSL_PSC_MDSTAT_EMUIHB_SHIFT                         (0x00000011U)
#define CSL_PSC_MDSTAT_EMUIHB_RESETVAL                      (0x00000000U)
#define CSL_PSC_MDSTAT_EMUIHB_MAX                           (0x00000001U)
/*----EMUIHB Tokens----*/
#define CSL_PSC_MDSTAT_EMUIHB_DISABLE           (0x00000000u)
#define CSL_PSC_MDSTAT_EMUIHB_ENABLE            (0x00000001u)

#define CSL_PSC_MDSTAT_EMURST_MASK                          (0x00010000U)
#define CSL_PSC_MDSTAT_EMURST_SHIFT                         (0x00000010U)
#define CSL_PSC_MDSTAT_EMURST_RESETVAL                      (0x00000000U)
#define CSL_PSC_MDSTAT_EMURST_MAX                           (0x00000001U)
/*----EMURST Tokens----*/
#define CSL_PSC_MDSTAT_EMURST_DISABLE           (0x00000000u)
#define CSL_PSC_MDSTAT_EMURST_ENABLE            (0x00000001u)

#define CSL_PSC_MDSTAT_MCKOUT_MASK                          (0x00001000U)
#define CSL_PSC_MDSTAT_MCKOUT_SHIFT                         (0x0000000CU)
#define CSL_PSC_MDSTAT_MCKOUT_RESETVAL                      (0x00000000U)
#define CSL_PSC_MDSTAT_MCKOUT_MAX                           (0x00000001U)
/*----MCKOUT Tokens----*/
#define CSL_PSC_MDSTAT_MCKOUT_OFF               (0x00000000u)
#define CSL_PSC_MDSTAT_MCKOUT_ON                (0x00000001u)

#define CSL_PSC_MDSTAT_MRSTDONE_MASK                        (0x00000800U)
#define CSL_PSC_MDSTAT_MRSTDONE_SHIFT                       (0x0000000BU)
#define CSL_PSC_MDSTAT_MRSTDONE_RESETVAL                    (0x00000000U)
#define CSL_PSC_MDSTAT_MRSTDONE_MAX                         (0x00000001U)
/*----MRSTDONE Tokens----*/
#define CSL_PSC_MDSTAT_MRSTDONE_COMPLETE        (0x00000000u)
#define CSL_PSC_MDSTAT_MRSTDONE_INCOMPLETE      (0x00000001u)

#define CSL_PSC_MDSTAT_MRST_MASK                            (0x00000400U)
#define CSL_PSC_MDSTAT_MRST_SHIFT                           (0x0000000AU)
#define CSL_PSC_MDSTAT_MRST_RESETVAL                        (0x00000000U)
#define CSL_PSC_MDSTAT_MRST_MAX                             (0x00000001U)
/*----MRST Tokens----*/
#define CSL_PSC_MDSTAT_MRST_ASSERT              (0x00000000u)
#define CSL_PSC_MDSTAT_MRST_DEASSERT            (0x00000001u)

#define CSL_PSC_MDSTAT_LRSTDONE_MASK                        (0x00000200U)
#define CSL_PSC_MDSTAT_LRSTDONE_SHIFT                       (0x00000009U)
#define CSL_PSC_MDSTAT_LRSTDONE_RESETVAL                    (0x00000000U)
#define CSL_PSC_MDSTAT_LRSTDONE_MAX                         (0x00000001U)
/*----LRSTDONE Tokens----*/
#define CSL_PSC_MDSTAT_LRSTDONE_NOTDONE         (0x00000000u)
#define CSL_PSC_MDSTAT_LRSTDONE_DONE            (0x00000001u)

#define CSL_PSC_MDSTAT_LRST_MASK                            (0x00000100U)
#define CSL_PSC_MDSTAT_LRST_SHIFT                           (0x00000008U)
#define CSL_PSC_MDSTAT_LRST_RESETVAL                        (0x00000000U)
#define CSL_PSC_MDSTAT_LRST_MAX                             (0x00000001U)
/*----LRST Tokens----*/
#define CSL_PSC_MDSTAT_LRST_ASSERT              (0x00000000u)
#define CSL_PSC_MDSTAT_LRST_DEASSERT            (0x00000001u)

#define CSL_PSC_MDSTAT_STATE_MASK                           (0x0000003FU)
#define CSL_PSC_MDSTAT_STATE_SHIFT                          (0x00000000U)
#define CSL_PSC_MDSTAT_STATE_RESETVAL                       (0x00000000U)
#define CSL_PSC_MDSTAT_STATE_MAX                            (0x0000003FU)
/*----STATE Tokens----*/
#define CSL_PSC_MDSTAT_STATE_SWRSTDISABLE       (0x00000000u)
#define CSL_PSC_MDSTAT_STATE_SYNCRST            (0x00000001u)
#define CSL_PSC_MDSTAT_STATE_DISABLE            (0x00000002u)
#define CSL_PSC_MDSTAT_STATE_ENABLE             (0x00000003u)
#define CSL_PSC_MDSTAT_STATE_AUTOSLEEP          (0x00000004u)
#define CSL_PSC_MDSTAT_STATE_AUTOWAKE           (0x00000005u)

#define CSL_PSC_MDSTAT_RESETVAL                             (0x00000000U)

/* MDCTL */

#define CSL_PSC_MDCTL_FORCE_MASK                            (0x80000000u)
#define CSL_PSC_MDCTL_FORCE_SHIFT                           (0x0000001Fu)
#define CSL_PSC_MDCTL_FORCE_RESETVAL                        (0x00000000u)
/*----FORCE Tokens----*/
#define CSL_PSC_MDCTL_FORCE_DISABLE             (0x00000000u)
#define CSL_PSC_MDCTL_FORCE_ENABLE              (0x00000001u)

#define CSL_PSC_MDCTL_RSTISO_MASK                           (0x00001000U)
#define CSL_PSC_MDCTL_RSTISO_SHIFT                          (0x0000000CU)
#define CSL_PSC_MDCTL_RSTISO_RESETVAL                       (0x00000000U)
#define CSL_PSC_MDCTL_RSTISO_MAX                            (0x00000001U)
/*----RSTISO Tokens----*/
#define CSL_PSC_MDCTL_RSTISO_DISABLE            (0x00000000u)
#define CSL_PSC_MDCTL_RSTISO_ENABLE             (0x00000001u)

#define CSL_PSC_MDCTL_BLKCHIP1RST_MASK                      (0x00000800U)
#define CSL_PSC_MDCTL_BLKCHIP1RST_SHIFT                     (0x0000000BU)
#define CSL_PSC_MDCTL_BLKCHIP1RST_RESETVAL                  (0x00000000U)
#define CSL_PSC_MDCTL_BLKCHIP1RST_MAX                       (0x00000001U)

#define CSL_PSC_MDCTL_EMUIHBIE_MASK                         (0x00000400u)
#define CSL_PSC_MDCTL_EMUIHBIE_SHIFT                        (0x0000000Au)
#define CSL_PSC_MDCTL_EMUIHBIE_RESETVAL                     (0x00000000u)
/*----EMUIHBIE Tokens----*/
#define CSL_PSC_MDCTL_EMUIHBIE_DISABLE          (0x00000000u)
#define CSL_PSC_MDCTL_EMUIHBIE_ENABLE           (0x00000001u)

#define CSL_PSC_MDCTL_EMURSTIE_MASK                         (0x00000200u)
#define CSL_PSC_MDCTL_EMURSTIE_SHIFT                        (0x00000009u)
#define CSL_PSC_MDCTL_EMURSTIE_RESETVAL                     (0x00000000u)
/*----EMURSTIE Tokens----*/
#define CSL_PSC_MDCTL_EMURSTIE_DISABLE          (0x00000000u)
#define CSL_PSC_MDCTL_EMURSTIE_ENABLE           (0x00000001u)

#define CSL_PSC_MDCTL_LRST_MASK                             (0x00000100U)
#define CSL_PSC_MDCTL_LRST_SHIFT                            (0x00000008U)
#define CSL_PSC_MDCTL_LRST_RESETVAL                         (0x00000000U)
#define CSL_PSC_MDCTL_LRST_MAX                              (0x00000001U)
/*----LRST Tokens----*/
#define CSL_PSC_MDCTL_LRST_ASSERT               (0x00000000u)
#define CSL_PSC_MDCTL_LRST_DEASSERT             (0x00000001u)

#define CSL_PSC_MDCTL_NEXT_MASK                             (0x0000001FU)
#define CSL_PSC_MDCTL_NEXT_SHIFT                            (0x00000000U)
#define CSL_PSC_MDCTL_NEXT_RESETVAL                         (0x00000001U)
#define CSL_PSC_MDCTL_NEXT_MAX                              (0x0000001FU)
/*----NEXT Tokens----*/
#define CSL_PSC_MDCTL_NEXT_SWRSTDISABLE         (0x00000000u)
#define CSL_PSC_MDCTL_NEXT_SYNCRST              (0x00000001u)
#define CSL_PSC_MDCTL_NEXT_DISABLE              (0x00000002u)
#define CSL_PSC_MDCTL_NEXT_ENABLE               (0x00000003u)
#define CSL_PSC_MDCTL_NEXT_AUTOSLEEP            (0x00000004u)
#define CSL_PSC_MDCTL_NEXT_AUTOWAKE             (0x00000005u)

#define CSL_PSC_MDCTL_RESETVAL                              (0x00000001U)


#ifdef __cplusplus
}
#endif
#endif
