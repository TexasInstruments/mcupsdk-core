/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_ottocal.h
*/
#ifndef CSLR_OTTOCAL_H_
#define CSLR_OTTOCAL_H_

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
    volatile uint8_t  Resv_66[66];
    volatile uint16_t HRPWR;
    volatile uint16_t HRCAL;
    volatile uint16_t HRPRD;
    volatile uint16_t HRCNT0;
    volatile uint16_t HRCNT1;
    volatile uint16_t HRMSTEP;
} CSL_ottocalRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_OTTOCAL_HRPWR                                                      (0x00000042U)
#define CSL_OTTOCAL_HRCAL                                                      (0x00000044U)
#define CSL_OTTOCAL_HRPRD                                                      (0x00000046U)
#define CSL_OTTOCAL_HRCNT0                                                     (0x00000048U)
#define CSL_OTTOCAL_HRCNT1                                                     (0x0000004AU)
#define CSL_OTTOCAL_HRMSTEP                                                    (0x0000004CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* HRPWR */

#define CSL_OTTOCAL_HRPWR_CALMODE_MASK                                         (0x0003U)
#define CSL_OTTOCAL_HRPWR_CALMODE_SHIFT                                        (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALMODE_RESETVAL                                     (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALMODE_MAX                                          (0x0003U)

#define CSL_OTTOCAL_HRPWR_CALSTART_MASK                                        (0x0004U)
#define CSL_OTTOCAL_HRPWR_CALSTART_SHIFT                                       (0x0002U)
#define CSL_OTTOCAL_HRPWR_CALSTART_RESETVAL                                    (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALSTART_MAX                                         (0x0001U)

#define CSL_OTTOCAL_HRPWR_CNTSEL_MASK                                          (0x0008U)
#define CSL_OTTOCAL_HRPWR_CNTSEL_SHIFT                                         (0x0003U)
#define CSL_OTTOCAL_HRPWR_CNTSEL_RESETVAL                                      (0x0000U)
#define CSL_OTTOCAL_HRPWR_CNTSEL_MAX                                           (0x0001U)

#define CSL_OTTOCAL_HRPWR_CALSTS_MASK                                          (0x0010U)
#define CSL_OTTOCAL_HRPWR_CALSTS_SHIFT                                         (0x0004U)
#define CSL_OTTOCAL_HRPWR_CALSTS_RESETVAL                                      (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALSTS_MAX                                           (0x0001U)

#define CSL_OTTOCAL_HRPWR_TESTSEL_MASK                                         (0x0020U)
#define CSL_OTTOCAL_HRPWR_TESTSEL_SHIFT                                        (0x0005U)
#define CSL_OTTOCAL_HRPWR_TESTSEL_RESETVAL                                     (0x0000U)
#define CSL_OTTOCAL_HRPWR_TESTSEL_MAX                                          (0x0001U)

#define CSL_OTTOCAL_HRPWR_CALSEL_MASK                                          (0x03C0U)
#define CSL_OTTOCAL_HRPWR_CALSEL_SHIFT                                         (0x0006U)
#define CSL_OTTOCAL_HRPWR_CALSEL_RESETVAL                                      (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALSEL_MAX                                           (0x000FU)

#define CSL_OTTOCAL_HRPWR_RESERVED_1_MASK                                      (0x7C00U)
#define CSL_OTTOCAL_HRPWR_RESERVED_1_SHIFT                                     (0x000AU)
#define CSL_OTTOCAL_HRPWR_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_OTTOCAL_HRPWR_RESERVED_1_MAX                                       (0x001FU)

#define CSL_OTTOCAL_HRPWR_CALPWRON_MASK                                        (0x8000U)
#define CSL_OTTOCAL_HRPWR_CALPWRON_SHIFT                                       (0x000FU)
#define CSL_OTTOCAL_HRPWR_CALPWRON_RESETVAL                                    (0x0000U)
#define CSL_OTTOCAL_HRPWR_CALPWRON_MAX                                         (0x0001U)

#define CSL_OTTOCAL_HRPWR_RESETVAL                                             (0x0000U)

/* HRCAL */

#define CSL_OTTOCAL_HRCAL_HRCAL_MASK                                           (0x00FFU)
#define CSL_OTTOCAL_HRCAL_HRCAL_SHIFT                                          (0x0000U)
#define CSL_OTTOCAL_HRCAL_HRCAL_RESETVAL                                       (0x0000U)
#define CSL_OTTOCAL_HRCAL_HRCAL_MAX                                            (0x00FFU)

#define CSL_OTTOCAL_HRCAL_RESERVED_1_MASK                                      (0xFF00U)
#define CSL_OTTOCAL_HRCAL_RESERVED_1_SHIFT                                     (0x0008U)
#define CSL_OTTOCAL_HRCAL_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_OTTOCAL_HRCAL_RESERVED_1_MAX                                       (0x00FFU)

#define CSL_OTTOCAL_HRCAL_RESETVAL                                             (0x0000U)

/* HRPRD */

#define CSL_OTTOCAL_HRPRD_HRPRD_MASK                                           (0xFFFFU)
#define CSL_OTTOCAL_HRPRD_HRPRD_SHIFT                                          (0x0000U)
#define CSL_OTTOCAL_HRPRD_HRPRD_RESETVAL                                       (0x0000U)
#define CSL_OTTOCAL_HRPRD_HRPRD_MAX                                            (0xFFFFU)

#define CSL_OTTOCAL_HRPRD_RESETVAL                                             (0x0000U)

/* HRCNT0 */

#define CSL_OTTOCAL_HRCNT0_HRCNT0_MASK                                         (0xFFFFU)
#define CSL_OTTOCAL_HRCNT0_HRCNT0_SHIFT                                        (0x0000U)
#define CSL_OTTOCAL_HRCNT0_HRCNT0_RESETVAL                                     (0x0000U)
#define CSL_OTTOCAL_HRCNT0_HRCNT0_MAX                                          (0xFFFFU)

#define CSL_OTTOCAL_HRCNT0_RESETVAL                                            (0x0000U)

/* HRCNT1 */

#define CSL_OTTOCAL_HRCNT1_HRCNT1_MASK                                         (0xFFFFU)
#define CSL_OTTOCAL_HRCNT1_HRCNT1_SHIFT                                        (0x0000U)
#define CSL_OTTOCAL_HRCNT1_HRCNT1_RESETVAL                                     (0x0000U)
#define CSL_OTTOCAL_HRCNT1_HRCNT1_MAX                                          (0xFFFFU)

#define CSL_OTTOCAL_HRCNT1_RESETVAL                                            (0x0000U)

/* HRMSTEP */

#define CSL_OTTOCAL_HRMSTEP_HRMSTEP_MASK                                       (0x00FFU)
#define CSL_OTTOCAL_HRMSTEP_HRMSTEP_SHIFT                                      (0x0000U)
#define CSL_OTTOCAL_HRMSTEP_HRMSTEP_RESETVAL                                   (0x0000U)
#define CSL_OTTOCAL_HRMSTEP_HRMSTEP_MAX                                        (0x00FFU)

#define CSL_OTTOCAL_HRMSTEP_RESERVED_1_MASK                                    (0xFF00U)
#define CSL_OTTOCAL_HRMSTEP_RESERVED_1_SHIFT                                   (0x0008U)
#define CSL_OTTOCAL_HRMSTEP_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_OTTOCAL_HRMSTEP_RESERVED_1_MAX                                     (0x00FFU)

#define CSL_OTTOCAL_HRMSTEP_RESETVAL                                           (0x0000U)

#ifdef __cplusplus
}
#endif
#endif
