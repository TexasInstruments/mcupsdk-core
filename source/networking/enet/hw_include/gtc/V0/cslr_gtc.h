/********************************************************************
 * Copyright (C) 2019 Texas Instruments Incorporated.
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
 *  Name        : cslr_gtc.h
*/
#ifndef CSLR_GTC_H_
#define CSLR_GTC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_GTC_CFG0_REGS_BASE                                             (0x00000000U)
#define CSL_GTC_CFG1_REGS_BASE                                             (0x00000000U)
#define CSL_GTC_CFG2_REGS_BASE                                             (0x00000000U)
#define CSL_GTC_CFG3_REGS_BASE                                             (0x00000000U)


/**************************************************************************
* Hardware Region  : MMRs in region 0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* PID register */
    volatile uint32_t GTC_PID;
    volatile uint32_t PUSHEVT;
} CSL_gtc_cfg0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GTC_CFG0_PID                                                   (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID                                               (0x00000004U)
#define CSL_GTC_CFG0_PUSHEVT                                               (0x00000008U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_GTC_CFG0_PID_MINOR_MASK                                        (0x0000003FU)
#define CSL_GTC_CFG0_PID_MINOR_SHIFT                                       (0x00000000U)
#define CSL_GTC_CFG0_PID_MINOR_RESETVAL                                    (0x00000010U)
#define CSL_GTC_CFG0_PID_MINOR_MAX                                         (0x0000003FU)

#define CSL_GTC_CFG0_PID_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_GTC_CFG0_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_GTC_CFG0_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_GTC_CFG0_PID_CUSTOM_MAX                                        (0x00000003U)

#define CSL_GTC_CFG0_PID_MAJOR_MASK                                        (0x00000700U)
#define CSL_GTC_CFG0_PID_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_GTC_CFG0_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define CSL_GTC_CFG0_PID_MAJOR_MAX                                         (0x00000007U)

#define CSL_GTC_CFG0_PID_MISC_MASK                                         (0x0000F800U)
#define CSL_GTC_CFG0_PID_MISC_SHIFT                                        (0x0000000BU)
#define CSL_GTC_CFG0_PID_MISC_RESETVAL                                     (0x00000000U)
#define CSL_GTC_CFG0_PID_MISC_MAX                                          (0x0000001FU)

#define CSL_GTC_CFG0_PID_MSB16_MASK                                        (0xFFFF0000U)
#define CSL_GTC_CFG0_PID_MSB16_SHIFT                                       (0x00000010U)
#define CSL_GTC_CFG0_PID_MSB16_RESETVAL                                    (0x00006180U)
#define CSL_GTC_CFG0_PID_MSB16_MAX                                         (0x0000FFFFU)

#define CSL_GTC_CFG0_PID_RESETVAL                                          (0x61800210U)

/* GTC_PID */

#define CSL_GTC_CFG0_GTC_PID_Y_MINOR_MASK                                  (0x0000003FU)
#define CSL_GTC_CFG0_GTC_PID_Y_MINOR_SHIFT                                 (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID_Y_MINOR_RESETVAL                              (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID_Y_MINOR_MAX                                   (0x0000003FU)

#define CSL_GTC_CFG0_GTC_PID_CUSTOM_MASK                                   (0x000000C0U)
#define CSL_GTC_CFG0_GTC_PID_CUSTOM_SHIFT                                  (0x00000006U)
#define CSL_GTC_CFG0_GTC_PID_CUSTOM_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID_CUSTOM_MAX                                    (0x00000003U)

#define CSL_GTC_CFG0_GTC_PID_X_MAJOR_MASK                                  (0x00000700U)
#define CSL_GTC_CFG0_GTC_PID_X_MAJOR_SHIFT                                 (0x00000008U)
#define CSL_GTC_CFG0_GTC_PID_X_MAJOR_RESETVAL                              (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID_X_MAJOR_MAX                                   (0x00000007U)

#define CSL_GTC_CFG0_GTC_PID_R_RTL_MASK                                    (0x0000F800U)
#define CSL_GTC_CFG0_GTC_PID_R_RTL_SHIFT                                   (0x0000000BU)
#define CSL_GTC_CFG0_GTC_PID_R_RTL_RESETVAL                                (0x00000000U)
#define CSL_GTC_CFG0_GTC_PID_R_RTL_MAX                                     (0x0000001FU)

#define CSL_GTC_CFG0_GTC_PID_FUNC_MASK                                     (0x0FFF0000U)
#define CSL_GTC_CFG0_GTC_PID_FUNC_SHIFT                                    (0x00000010U)
#define CSL_GTC_CFG0_GTC_PID_FUNC_RESETVAL                                 (0x0000061AU)
#define CSL_GTC_CFG0_GTC_PID_FUNC_MAX                                      (0x00000FFFU)

#define CSL_GTC_CFG0_GTC_PID_BU_MASK                                       (0x30000000U)
#define CSL_GTC_CFG0_GTC_PID_BU_SHIFT                                      (0x0000001CU)
#define CSL_GTC_CFG0_GTC_PID_BU_RESETVAL                                   (0x00000002U)
#define CSL_GTC_CFG0_GTC_PID_BU_MAX                                        (0x00000003U)

#define CSL_GTC_CFG0_GTC_PID_SCHEME_MASK                                   (0xC0000000U)
#define CSL_GTC_CFG0_GTC_PID_SCHEME_SHIFT                                  (0x0000001EU)
#define CSL_GTC_CFG0_GTC_PID_SCHEME_RESETVAL                               (0x00000001U)
#define CSL_GTC_CFG0_GTC_PID_SCHEME_MAX                                    (0x00000003U)

#define CSL_GTC_CFG0_GTC_PID_RESETVAL                                      (0x661A0000U)

/* PUSHEVT */

#define CSL_GTC_CFG0_PUSHEVT_EXPBIT_SEL_MASK                               (0x0000003FU)
#define CSL_GTC_CFG0_PUSHEVT_EXPBIT_SEL_SHIFT                              (0x00000000U)
#define CSL_GTC_CFG0_PUSHEVT_EXPBIT_SEL_RESETVAL                           (0x00000000U)
#define CSL_GTC_CFG0_PUSHEVT_EXPBIT_SEL_MAX                                (0x0000003FU)

#define CSL_GTC_CFG0_PUSHEVT_RESETVAL                                      (0x00000000U)

/**************************************************************************
* Hardware Region  : MMRs in region 1
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CNTCR;
    volatile uint32_t CNTSR;
    volatile uint32_t CNTCV_LO;
    volatile uint32_t CNTCV_HI;
    volatile uint8_t  Resv_32[16];
    volatile uint32_t CNTFID0;
    volatile uint32_t CNTFID1;
} CSL_gtc_cfg1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GTC_CFG1_CNTCR                                                 (0x00000000U)
#define CSL_GTC_CFG1_CNTSR                                                 (0x00000004U)
#define CSL_GTC_CFG1_CNTCV_LO                                              (0x00000008U)
#define CSL_GTC_CFG1_CNTCV_HI                                              (0x0000000CU)
#define CSL_GTC_CFG1_CNTFID0                                               (0x00000020U)
#define CSL_GTC_CFG1_CNTFID1                                               (0x00000024U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CNTCR */

#define CSL_GTC_CFG1_CNTCR_EN_MASK                                         (0x00000001U)
#define CSL_GTC_CFG1_CNTCR_EN_SHIFT                                        (0x00000000U)
#define CSL_GTC_CFG1_CNTCR_EN_RESETVAL                                     (0x00000000U)
#define CSL_GTC_CFG1_CNTCR_EN_MAX                                          (0x00000001U)

#define CSL_GTC_CFG1_CNTCR_HDBG_MASK                                       (0x00000002U)
#define CSL_GTC_CFG1_CNTCR_HDBG_SHIFT                                      (0x00000001U)
#define CSL_GTC_CFG1_CNTCR_HDBG_RESETVAL                                   (0x00000000U)
#define CSL_GTC_CFG1_CNTCR_HDBG_MAX                                        (0x00000001U)

#define CSL_GTC_CFG1_CNTCR_FCREQ_MASK                                      (0xFFFFFF00U)
#define CSL_GTC_CFG1_CNTCR_FCREQ_SHIFT                                     (0x00000008U)
#define CSL_GTC_CFG1_CNTCR_FCREQ_RESETVAL                                  (0x00000000U)
#define CSL_GTC_CFG1_CNTCR_FCREQ_MAX                                       (0x00FFFFFFU)

#define CSL_GTC_CFG1_CNTCR_RESETVAL                                        (0x00000000U)

/* CNTSR */

#define CSL_GTC_CFG1_CNTSR_DBGH_MASK                                       (0x00000002U)
#define CSL_GTC_CFG1_CNTSR_DBGH_SHIFT                                      (0x00000001U)
#define CSL_GTC_CFG1_CNTSR_DBGH_RESETVAL                                   (0x00000000U)
#define CSL_GTC_CFG1_CNTSR_DBGH_MAX                                        (0x00000001U)

#define CSL_GTC_CFG1_CNTSR_FCACK_MASK                                      (0xFFFFFF00U)
#define CSL_GTC_CFG1_CNTSR_FCACK_SHIFT                                     (0x00000008U)
#define CSL_GTC_CFG1_CNTSR_FCACK_RESETVAL                                  (0x00000000U)
#define CSL_GTC_CFG1_CNTSR_FCACK_MAX                                       (0x00FFFFFFU)

#define CSL_GTC_CFG1_CNTSR_RESETVAL                                        (0x00000000U)

/* CNTCV_LO */

#define CSL_GTC_CFG1_CNTCV_LO_COUNTVALUE_MASK                              (0xFFFFFFFFU)
#define CSL_GTC_CFG1_CNTCV_LO_COUNTVALUE_SHIFT                             (0x00000000U)
#define CSL_GTC_CFG1_CNTCV_LO_COUNTVALUE_RESETVAL                          (0x00000000U)
#define CSL_GTC_CFG1_CNTCV_LO_COUNTVALUE_MAX                               (0xFFFFFFFFU)

#define CSL_GTC_CFG1_CNTCV_LO_RESETVAL                                     (0x00000000U)

/* CNTCV_HI */

#define CSL_GTC_CFG1_CNTCV_HI_COUNTVALUE_MASK                              (0xFFFFFFFFU)
#define CSL_GTC_CFG1_CNTCV_HI_COUNTVALUE_SHIFT                             (0x00000000U)
#define CSL_GTC_CFG1_CNTCV_HI_COUNTVALUE_RESETVAL                          (0x00000000U)
#define CSL_GTC_CFG1_CNTCV_HI_COUNTVALUE_MAX                               (0xFFFFFFFFU)

#define CSL_GTC_CFG1_CNTCV_HI_RESETVAL                                     (0x00000000U)

/* CNTFID0 */

#define CSL_GTC_CFG1_CNTFID0_FREQVALUE_MASK                                (0xFFFFFFFFU)
#define CSL_GTC_CFG1_CNTFID0_FREQVALUE_SHIFT                               (0x00000000U)
#define CSL_GTC_CFG1_CNTFID0_FREQVALUE_RESETVAL                            (0x00000000U)
#define CSL_GTC_CFG1_CNTFID0_FREQVALUE_MAX                                 (0xFFFFFFFFU)

#define CSL_GTC_CFG1_CNTFID0_RESETVAL                                      (0x00000000U)

/* CNTFID1 */

#define CSL_GTC_CFG1_CNTFID1_FREQVALUE_MASK                                (0xFFFFFFFFU)
#define CSL_GTC_CFG1_CNTFID1_FREQVALUE_SHIFT                               (0x00000000U)
#define CSL_GTC_CFG1_CNTFID1_FREQVALUE_RESETVAL                            (0x00000000U)
#define CSL_GTC_CFG1_CNTFID1_FREQVALUE_MAX                                 (0xFFFFFFFFU)

#define CSL_GTC_CFG1_CNTFID1_RESETVAL                                      (0x00000000U)

/**************************************************************************
* Hardware Region  : MMRs in region 2
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CNTCVS_LO;
    volatile uint32_t CNTCVS_HI;
} CSL_gtc_cfg2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GTC_CFG2_CNTCVS_LO                                             (0x00000000U)
#define CSL_GTC_CFG2_CNTCVS_HI                                             (0x00000004U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CNTCVS_LO */

#define CSL_GTC_CFG2_CNTCVS_LO_COUNTVALUE_MASK                             (0xFFFFFFFFU)
#define CSL_GTC_CFG2_CNTCVS_LO_COUNTVALUE_SHIFT                            (0x00000000U)
#define CSL_GTC_CFG2_CNTCVS_LO_COUNTVALUE_RESETVAL                         (0x00000000U)
#define CSL_GTC_CFG2_CNTCVS_LO_COUNTVALUE_MAX                              (0xFFFFFFFFU)

#define CSL_GTC_CFG2_CNTCVS_LO_RESETVAL                                    (0x00000000U)

/* CNTCVS_HI */

#define CSL_GTC_CFG2_CNTCVS_HI_COUNTVALUE_MASK                             (0xFFFFFFFFU)
#define CSL_GTC_CFG2_CNTCVS_HI_COUNTVALUE_SHIFT                            (0x00000000U)
#define CSL_GTC_CFG2_CNTCVS_HI_COUNTVALUE_RESETVAL                         (0x00000000U)
#define CSL_GTC_CFG2_CNTCVS_HI_COUNTVALUE_MAX                              (0xFFFFFFFFU)

#define CSL_GTC_CFG2_CNTCVS_HI_RESETVAL                                    (0x00000000U)

/**************************************************************************
* Hardware Region  : MMRs in region 3
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_8[8];
    volatile uint32_t CNTTIDR;
} CSL_gtc_cfg3Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GTC_CFG3_CNTTIDR                                               (0x00000008U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CNTTIDR */

#define CSL_GTC_CFG3_CNTTIDR_FRAME0_MASK                                   (0x0000000FU)
#define CSL_GTC_CFG3_CNTTIDR_FRAME0_SHIFT                                  (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME0_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME0_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME1_MASK                                   (0x000000F0U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME1_SHIFT                                  (0x00000004U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME1_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME1_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME2_MASK                                   (0x00000F00U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME2_SHIFT                                  (0x00000008U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME2_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME2_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME3_MASK                                   (0x0000F000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME3_SHIFT                                  (0x0000000CU)
#define CSL_GTC_CFG3_CNTTIDR_FRAME3_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME3_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME4_MASK                                   (0x000F0000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME4_SHIFT                                  (0x00000010U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME4_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME4_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME5_MASK                                   (0x00F00000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME5_SHIFT                                  (0x00000014U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME5_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME5_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME6_MASK                                   (0x0F000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME6_SHIFT                                  (0x00000018U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME6_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME6_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_FRAME7_MASK                                   (0xF0000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME7_SHIFT                                  (0x0000001CU)
#define CSL_GTC_CFG3_CNTTIDR_FRAME7_RESETVAL                               (0x00000000U)
#define CSL_GTC_CFG3_CNTTIDR_FRAME7_MAX                                    (0x0000000FU)

#define CSL_GTC_CFG3_CNTTIDR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
