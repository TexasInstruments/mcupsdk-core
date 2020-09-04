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
 *  Name        : cslr_cmpssa.h
*/
#ifndef CSLR_CMPSSA_H_
#define CSLR_CMPSSA_H_

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
    volatile uint16_t COMPCTL;
    volatile uint16_t COMPHYSCTL;
    volatile uint16_t COMPSTS;
    volatile uint16_t COMPSTSCLR;
    volatile uint16_t COMPDACCTL;
    volatile uint16_t COMPDACCTL2;
    volatile uint16_t DACHVALS;
    volatile uint16_t DACHVALA;
    volatile uint16_t RAMPMAXREFA;
    volatile uint8_t  Resv_20[2];
    volatile uint16_t RAMPMAXREFS;
    volatile uint8_t  Resv_24[2];
    volatile uint16_t RAMPDECVALA;
    volatile uint8_t  Resv_28[2];
    volatile uint16_t RAMPDECVALS;
    volatile uint8_t  Resv_32[2];
    volatile uint16_t RAMPSTS;
    volatile uint8_t  Resv_36[2];
    volatile uint16_t DACLVALS;
    volatile uint16_t DACLVALA;
    volatile uint16_t RAMPDLYA;
    volatile uint16_t RAMPDLYS;
    volatile uint16_t CTRIPLFILCTL;
    volatile uint16_t CTRIPLFILCLKCTL;
    volatile uint16_t CTRIPHFILCTL;
    volatile uint16_t CTRIPHFILCLKCTL;
    volatile uint16_t COMPLOCK;
    volatile uint8_t  Resv_56[2];
    volatile uint16_t DACHVALS2;
    volatile uint16_t DACLVALS2;
    volatile uint16_t CONFIG1;
} CSL_cmpssaRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CMPSSA_COMPCTL                                                     (0x00000000U)
#define CSL_CMPSSA_COMPHYSCTL                                                  (0x00000002U)
#define CSL_CMPSSA_COMPSTS                                                     (0x00000004U)
#define CSL_CMPSSA_COMPSTSCLR                                                  (0x00000006U)
#define CSL_CMPSSA_COMPDACCTL                                                  (0x00000008U)
#define CSL_CMPSSA_COMPDACCTL2                                                 (0x0000000AU)
#define CSL_CMPSSA_DACHVALS                                                    (0x0000000CU)
#define CSL_CMPSSA_DACHVALA                                                    (0x0000000EU)
#define CSL_CMPSSA_RAMPMAXREFA                                                 (0x00000010U)
#define CSL_CMPSSA_RAMPMAXREFS                                                 (0x00000014U)
#define CSL_CMPSSA_RAMPDECVALA                                                 (0x00000018U)
#define CSL_CMPSSA_RAMPDECVALS                                                 (0x0000001CU)
#define CSL_CMPSSA_RAMPSTS                                                     (0x00000020U)
#define CSL_CMPSSA_DACLVALS                                                    (0x00000024U)
#define CSL_CMPSSA_DACLVALA                                                    (0x00000026U)
#define CSL_CMPSSA_RAMPDLYA                                                    (0x00000028U)
#define CSL_CMPSSA_RAMPDLYS                                                    (0x0000002AU)
#define CSL_CMPSSA_CTRIPLFILCTL                                                (0x0000002CU)
#define CSL_CMPSSA_CTRIPLFILCLKCTL                                             (0x0000002EU)
#define CSL_CMPSSA_CTRIPHFILCTL                                                (0x00000030U)
#define CSL_CMPSSA_CTRIPHFILCLKCTL                                             (0x00000032U)
#define CSL_CMPSSA_COMPLOCK                                                    (0x00000034U)
#define CSL_CMPSSA_DACHVALS2                                                   (0x00000038U)
#define CSL_CMPSSA_DACLVALS2                                                   (0x0000003AU)
#define CSL_CMPSSA_CONFIG1                                                     (0x0000003CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* COMPCTL */

#define CSL_CMPSSA_COMPCTL_COMPHSOURCE_MASK                                    (0x0001U)
#define CSL_CMPSSA_COMPCTL_COMPHSOURCE_SHIFT                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPHSOURCE_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPHSOURCE_MAX                                     (0x0001U)

#define CSL_CMPSSA_COMPCTL_COMPHINV_MASK                                       (0x0002U)
#define CSL_CMPSSA_COMPCTL_COMPHINV_SHIFT                                      (0x0001U)
#define CSL_CMPSSA_COMPCTL_COMPHINV_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPHINV_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPCTL_CTRIPHSEL_MASK                                      (0x000CU)
#define CSL_CMPSSA_COMPCTL_CTRIPHSEL_SHIFT                                     (0x0002U)
#define CSL_CMPSSA_COMPCTL_CTRIPHSEL_RESETVAL                                  (0x0000U)
#define CSL_CMPSSA_COMPCTL_CTRIPHSEL_MAX                                       (0x0003U)

#define CSL_CMPSSA_COMPCTL_CTRIPOUTHSEL_MASK                                   (0x0030U)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTHSEL_SHIFT                                  (0x0004U)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTHSEL_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTHSEL_MAX                                    (0x0003U)

#define CSL_CMPSSA_COMPCTL_ASYNCHEN_MASK                                       (0x0040U)
#define CSL_CMPSSA_COMPCTL_ASYNCHEN_SHIFT                                      (0x0006U)
#define CSL_CMPSSA_COMPCTL_ASYNCHEN_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_ASYNCHEN_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPCTL_RESERVED_1_MASK                                     (0x0080U)
#define CSL_CMPSSA_COMPCTL_RESERVED_1_SHIFT                                    (0x0007U)
#define CSL_CMPSSA_COMPCTL_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPCTL_RESERVED_1_MAX                                      (0x0001U)

#define CSL_CMPSSA_COMPCTL_COMPLSOURCE_MASK                                    (0x0100U)
#define CSL_CMPSSA_COMPCTL_COMPLSOURCE_SHIFT                                   (0x0008U)
#define CSL_CMPSSA_COMPCTL_COMPLSOURCE_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPLSOURCE_MAX                                     (0x0001U)

#define CSL_CMPSSA_COMPCTL_COMPLINV_MASK                                       (0x0200U)
#define CSL_CMPSSA_COMPCTL_COMPLINV_SHIFT                                      (0x0009U)
#define CSL_CMPSSA_COMPCTL_COMPLINV_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPLINV_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPCTL_CTRIPLSEL_MASK                                      (0x0C00U)
#define CSL_CMPSSA_COMPCTL_CTRIPLSEL_SHIFT                                     (0x000AU)
#define CSL_CMPSSA_COMPCTL_CTRIPLSEL_RESETVAL                                  (0x0000U)
#define CSL_CMPSSA_COMPCTL_CTRIPLSEL_MAX                                       (0x0003U)

#define CSL_CMPSSA_COMPCTL_CTRIPOUTLSEL_MASK                                   (0x3000U)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTLSEL_SHIFT                                  (0x000CU)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTLSEL_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPCTL_CTRIPOUTLSEL_MAX                                    (0x0003U)

#define CSL_CMPSSA_COMPCTL_ASYNCLEN_MASK                                       (0x4000U)
#define CSL_CMPSSA_COMPCTL_ASYNCLEN_SHIFT                                      (0x000EU)
#define CSL_CMPSSA_COMPCTL_ASYNCLEN_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_ASYNCLEN_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPCTL_COMPDACE_MASK                                       (0x8000U)
#define CSL_CMPSSA_COMPCTL_COMPDACE_SHIFT                                      (0x000FU)
#define CSL_CMPSSA_COMPCTL_COMPDACE_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPCTL_COMPDACE_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPCTL_RESETVAL                                            (0x0000U)

/* COMPHYSCTL */

#define CSL_CMPSSA_COMPHYSCTL_COMPHYS_MASK                                     (0x000FU)
#define CSL_CMPSSA_COMPHYSCTL_COMPHYS_SHIFT                                    (0x0000U)
#define CSL_CMPSSA_COMPHYSCTL_COMPHYS_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPHYSCTL_COMPHYS_MAX                                      (0x000FU)

#define CSL_CMPSSA_COMPHYSCTL_RESERVED_1_MASK                                  (0xFFF0U)
#define CSL_CMPSSA_COMPHYSCTL_RESERVED_1_SHIFT                                 (0x0004U)
#define CSL_CMPSSA_COMPHYSCTL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPHYSCTL_RESERVED_1_MAX                                   (0x0FFFU)

#define CSL_CMPSSA_COMPHYSCTL_RESETVAL                                         (0x0000U)

/* COMPSTS */

#define CSL_CMPSSA_COMPSTS_COMPHSTS_MASK                                       (0x0001U)
#define CSL_CMPSSA_COMPSTS_COMPHSTS_SHIFT                                      (0x0000U)
#define CSL_CMPSSA_COMPSTS_COMPHSTS_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPSTS_COMPHSTS_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPSTS_COMPHLATCH_MASK                                     (0x0002U)
#define CSL_CMPSSA_COMPSTS_COMPHLATCH_SHIFT                                    (0x0001U)
#define CSL_CMPSSA_COMPSTS_COMPHLATCH_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPSTS_COMPHLATCH_MAX                                      (0x0001U)

#define CSL_CMPSSA_COMPSTS_RESERVED_1_MASK                                     (0x00FCU)
#define CSL_CMPSSA_COMPSTS_RESERVED_1_SHIFT                                    (0x0002U)
#define CSL_CMPSSA_COMPSTS_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPSTS_RESERVED_1_MAX                                      (0x003FU)

#define CSL_CMPSSA_COMPSTS_COMPLSTS_MASK                                       (0x0100U)
#define CSL_CMPSSA_COMPSTS_COMPLSTS_SHIFT                                      (0x0008U)
#define CSL_CMPSSA_COMPSTS_COMPLSTS_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPSTS_COMPLSTS_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPSTS_COMPLLATCH_MASK                                     (0x0200U)
#define CSL_CMPSSA_COMPSTS_COMPLLATCH_SHIFT                                    (0x0009U)
#define CSL_CMPSSA_COMPSTS_COMPLLATCH_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPSTS_COMPLLATCH_MAX                                      (0x0001U)

#define CSL_CMPSSA_COMPSTS_RESERVED_2_MASK                                     (0xFC00U)
#define CSL_CMPSSA_COMPSTS_RESERVED_2_SHIFT                                    (0x000AU)
#define CSL_CMPSSA_COMPSTS_RESERVED_2_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPSTS_RESERVED_2_MAX                                      (0x003FU)

#define CSL_CMPSSA_COMPSTS_RESETVAL                                            (0x0000U)

/* COMPSTSCLR */

#define CSL_CMPSSA_COMPSTSCLR_RESERVED_1_MASK                                  (0x0001U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_1_SHIFT                                 (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_1_MAX                                   (0x0001U)

#define CSL_CMPSSA_COMPSTSCLR_HLATCHCLR_MASK                                   (0x0002U)
#define CSL_CMPSSA_COMPSTSCLR_HLATCHCLR_SHIFT                                  (0x0001U)
#define CSL_CMPSSA_COMPSTSCLR_HLATCHCLR_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_HLATCHCLR_MAX                                    (0x0001U)

#define CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_MASK                                  (0x0004U)
#define CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_SHIFT                                 (0x0002U)
#define CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_HSYNCCLREN_MAX                                   (0x0001U)

#define CSL_CMPSSA_COMPSTSCLR_RESERVED_2_MASK                                  (0x01F8U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_2_SHIFT                                 (0x0003U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_2_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_2_MAX                                   (0x003FU)

#define CSL_CMPSSA_COMPSTSCLR_LLATCHCLR_MASK                                   (0x0200U)
#define CSL_CMPSSA_COMPSTSCLR_LLATCHCLR_SHIFT                                  (0x0009U)
#define CSL_CMPSSA_COMPSTSCLR_LLATCHCLR_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_LLATCHCLR_MAX                                    (0x0001U)

#define CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_MASK                                  (0x0400U)
#define CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_SHIFT                                 (0x000AU)
#define CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_LSYNCCLREN_MAX                                   (0x0001U)

#define CSL_CMPSSA_COMPSTSCLR_RESERVED_3_MASK                                  (0xF800U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_3_SHIFT                                 (0x000BU)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_3_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPSTSCLR_RESERVED_3_MAX                                   (0x001FU)

#define CSL_CMPSSA_COMPSTSCLR_RESETVAL                                         (0x0000U)

/* COMPDACCTL */

#define CSL_CMPSSA_COMPDACCTL_DACSOURCE_MASK                                   (0x0001U)
#define CSL_CMPSSA_COMPDACCTL_DACSOURCE_SHIFT                                  (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_DACSOURCE_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_DACSOURCE_MAX                                    (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_MASK                                  (0x001EU)
#define CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_SHIFT                                 (0x0001U)
#define CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_RAMPSOURCE_MAX                                   (0x000FU)

#define CSL_CMPSSA_COMPDACCTL_SELREF_MASK                                      (0x0020U)
#define CSL_CMPSSA_COMPDACCTL_SELREF_SHIFT                                     (0x0005U)
#define CSL_CMPSSA_COMPDACCTL_SELREF_RESETVAL                                  (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_SELREF_MAX                                       (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_MASK                                 (0x0040U)
#define CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_SHIFT                                (0x0006U)
#define CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_RAMPLOADSEL_MAX                                  (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_SWLOADSEL_MASK                                   (0x0080U)
#define CSL_CMPSSA_COMPDACCTL_SWLOADSEL_SHIFT                                  (0x0007U)
#define CSL_CMPSSA_COMPDACCTL_SWLOADSEL_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_SWLOADSEL_MAX                                    (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_BLANKSOURCE_MASK                                 (0x0F00U)
#define CSL_CMPSSA_COMPDACCTL_BLANKSOURCE_SHIFT                                (0x0008U)
#define CSL_CMPSSA_COMPDACCTL_BLANKSOURCE_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_BLANKSOURCE_MAX                                  (0x000FU)

#define CSL_CMPSSA_COMPDACCTL_BLANKEN_MASK                                     (0x1000U)
#define CSL_CMPSSA_COMPDACCTL_BLANKEN_SHIFT                                    (0x000CU)
#define CSL_CMPSSA_COMPDACCTL_BLANKEN_RESETVAL                                 (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_BLANKEN_MAX                                      (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_RESERVED_1_MASK                                  (0x2000U)
#define CSL_CMPSSA_COMPDACCTL_RESERVED_1_SHIFT                                 (0x000DU)
#define CSL_CMPSSA_COMPDACCTL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_RESERVED_1_MAX                                   (0x0001U)

#define CSL_CMPSSA_COMPDACCTL_FREESOFT_MASK                                    (0xC000U)
#define CSL_CMPSSA_COMPDACCTL_FREESOFT_SHIFT                                   (0x000EU)
#define CSL_CMPSSA_COMPDACCTL_FREESOFT_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_COMPDACCTL_FREESOFT_MAX                                     (0x0003U)

#define CSL_CMPSSA_COMPDACCTL_RESETVAL                                         (0x0000U)

/* COMPDACCTL2 */

#define CSL_CMPSSA_COMPDACCTL2_DEENABLE_MASK                                   (0x0001U)
#define CSL_CMPSSA_COMPDACCTL2_DEENABLE_SHIFT                                  (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_DEENABLE_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_DEENABLE_MAX                                    (0x0001U)

#define CSL_CMPSSA_COMPDACCTL2_DEACTIVESEL_MASK                                (0x003EU)
#define CSL_CMPSSA_COMPDACCTL2_DEACTIVESEL_SHIFT                               (0x0001U)
#define CSL_CMPSSA_COMPDACCTL2_DEACTIVESEL_RESETVAL                            (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_DEACTIVESEL_MAX                                 (0x001FU)

#define CSL_CMPSSA_COMPDACCTL2_RESERVED_1_MASK                                 (0x00C0U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_1_SHIFT                                (0x0006U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_1_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_1_MAX                                  (0x0003U)

#define CSL_CMPSSA_COMPDACCTL2_BLANKSOURCEUSEL_MASK                            (0x0100U)
#define CSL_CMPSSA_COMPDACCTL2_BLANKSOURCEUSEL_SHIFT                           (0x0008U)
#define CSL_CMPSSA_COMPDACCTL2_BLANKSOURCEUSEL_RESETVAL                        (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_BLANKSOURCEUSEL_MAX                             (0x0001U)

#define CSL_CMPSSA_COMPDACCTL2_RESERVED_2_MASK                                 (0x0200U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_2_SHIFT                                (0x0009U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_2_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_2_MAX                                  (0x0001U)

#define CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_MASK                             (0x0400U)
#define CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_SHIFT                            (0x000AU)
#define CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_RESETVAL                         (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_RAMPSOURCEUSEL_MAX                              (0x0001U)

#define CSL_CMPSSA_COMPDACCTL2_RESERVED_3_MASK                                 (0xF800U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_3_SHIFT                                (0x000BU)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_3_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_COMPDACCTL2_RESERVED_3_MAX                                  (0x001FU)

#define CSL_CMPSSA_COMPDACCTL2_RESETVAL                                        (0x0000U)

/* DACHVALS */

#define CSL_CMPSSA_DACHVALS_DACVAL_MASK                                        (0x0FFFU)
#define CSL_CMPSSA_DACHVALS_DACVAL_SHIFT                                       (0x0000U)
#define CSL_CMPSSA_DACHVALS_DACVAL_RESETVAL                                    (0x0000U)
#define CSL_CMPSSA_DACHVALS_DACVAL_MAX                                         (0x0FFFU)

#define CSL_CMPSSA_DACHVALS_RESERVED_1_MASK                                    (0xF000U)
#define CSL_CMPSSA_DACHVALS_RESERVED_1_SHIFT                                   (0x000CU)
#define CSL_CMPSSA_DACHVALS_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_DACHVALS_RESERVED_1_MAX                                     (0x000FU)

#define CSL_CMPSSA_DACHVALS_RESETVAL                                           (0x0000U)

/* DACHVALA */

#define CSL_CMPSSA_DACHVALA_DACVAL_MASK                                        (0x0FFFU)
#define CSL_CMPSSA_DACHVALA_DACVAL_SHIFT                                       (0x0000U)
#define CSL_CMPSSA_DACHVALA_DACVAL_RESETVAL                                    (0x0000U)
#define CSL_CMPSSA_DACHVALA_DACVAL_MAX                                         (0x0FFFU)

#define CSL_CMPSSA_DACHVALA_RESERVED_1_MASK                                    (0xF000U)
#define CSL_CMPSSA_DACHVALA_RESERVED_1_SHIFT                                   (0x000CU)
#define CSL_CMPSSA_DACHVALA_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_DACHVALA_RESERVED_1_MAX                                     (0x000FU)

#define CSL_CMPSSA_DACHVALA_RESETVAL                                           (0x0000U)

/* RAMPMAXREFA */

#define CSL_CMPSSA_RAMPMAXREFA_RAMPMAXREF_MASK                                 (0xFFFFU)
#define CSL_CMPSSA_RAMPMAXREFA_RAMPMAXREF_SHIFT                                (0x0000U)
#define CSL_CMPSSA_RAMPMAXREFA_RAMPMAXREF_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_RAMPMAXREFA_RAMPMAXREF_MAX                                  (0xFFFFU)

#define CSL_CMPSSA_RAMPMAXREFA_RESETVAL                                        (0x0000U)

/* RAMPMAXREFS */

#define CSL_CMPSSA_RAMPMAXREFS_RAMPMAXREF_MASK                                 (0xFFFFU)
#define CSL_CMPSSA_RAMPMAXREFS_RAMPMAXREF_SHIFT                                (0x0000U)
#define CSL_CMPSSA_RAMPMAXREFS_RAMPMAXREF_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_RAMPMAXREFS_RAMPMAXREF_MAX                                  (0xFFFFU)

#define CSL_CMPSSA_RAMPMAXREFS_RESETVAL                                        (0x0000U)

/* RAMPDECVALA */

#define CSL_CMPSSA_RAMPDECVALA_RAMPDECVAL_MASK                                 (0xFFFFU)
#define CSL_CMPSSA_RAMPDECVALA_RAMPDECVAL_SHIFT                                (0x0000U)
#define CSL_CMPSSA_RAMPDECVALA_RAMPDECVAL_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_RAMPDECVALA_RAMPDECVAL_MAX                                  (0xFFFFU)

#define CSL_CMPSSA_RAMPDECVALA_RESETVAL                                        (0x0000U)

/* RAMPDECVALS */

#define CSL_CMPSSA_RAMPDECVALS_RAMPDECVAL_MASK                                 (0xFFFFU)
#define CSL_CMPSSA_RAMPDECVALS_RAMPDECVAL_SHIFT                                (0x0000U)
#define CSL_CMPSSA_RAMPDECVALS_RAMPDECVAL_RESETVAL                             (0x0000U)
#define CSL_CMPSSA_RAMPDECVALS_RAMPDECVAL_MAX                                  (0xFFFFU)

#define CSL_CMPSSA_RAMPDECVALS_RESETVAL                                        (0x0000U)

/* RAMPSTS */

#define CSL_CMPSSA_RAMPSTS_RAMPVALUE_MASK                                      (0xFFFFU)
#define CSL_CMPSSA_RAMPSTS_RAMPVALUE_SHIFT                                     (0x0000U)
#define CSL_CMPSSA_RAMPSTS_RAMPVALUE_RESETVAL                                  (0x0000U)
#define CSL_CMPSSA_RAMPSTS_RAMPVALUE_MAX                                       (0xFFFFU)

#define CSL_CMPSSA_RAMPSTS_RESETVAL                                            (0x0000U)

/* DACLVALS */

#define CSL_CMPSSA_DACLVALS_DACVAL_MASK                                        (0x0FFFU)
#define CSL_CMPSSA_DACLVALS_DACVAL_SHIFT                                       (0x0000U)
#define CSL_CMPSSA_DACLVALS_DACVAL_RESETVAL                                    (0x0000U)
#define CSL_CMPSSA_DACLVALS_DACVAL_MAX                                         (0x0FFFU)

#define CSL_CMPSSA_DACLVALS_RESERVED_1_MASK                                    (0xF000U)
#define CSL_CMPSSA_DACLVALS_RESERVED_1_SHIFT                                   (0x000CU)
#define CSL_CMPSSA_DACLVALS_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_DACLVALS_RESERVED_1_MAX                                     (0x000FU)

#define CSL_CMPSSA_DACLVALS_RESETVAL                                           (0x0000U)

/* DACLVALA */

#define CSL_CMPSSA_DACLVALA_DACVAL_MASK                                        (0x0FFFU)
#define CSL_CMPSSA_DACLVALA_DACVAL_SHIFT                                       (0x0000U)
#define CSL_CMPSSA_DACLVALA_DACVAL_RESETVAL                                    (0x0000U)
#define CSL_CMPSSA_DACLVALA_DACVAL_MAX                                         (0x0FFFU)

#define CSL_CMPSSA_DACLVALA_RESERVED_1_MASK                                    (0xF000U)
#define CSL_CMPSSA_DACLVALA_RESERVED_1_SHIFT                                   (0x000CU)
#define CSL_CMPSSA_DACLVALA_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_DACLVALA_RESERVED_1_MAX                                     (0x000FU)

#define CSL_CMPSSA_DACLVALA_RESETVAL                                           (0x0000U)

/* RAMPDLYA */

#define CSL_CMPSSA_RAMPDLYA_DELAY_MASK                                         (0x1FFFU)
#define CSL_CMPSSA_RAMPDLYA_DELAY_SHIFT                                        (0x0000U)
#define CSL_CMPSSA_RAMPDLYA_DELAY_RESETVAL                                     (0x0000U)
#define CSL_CMPSSA_RAMPDLYA_DELAY_MAX                                          (0x1FFFU)

#define CSL_CMPSSA_RAMPDLYA_RESERVED_1_MASK                                    (0xE000U)
#define CSL_CMPSSA_RAMPDLYA_RESERVED_1_SHIFT                                   (0x000DU)
#define CSL_CMPSSA_RAMPDLYA_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_RAMPDLYA_RESERVED_1_MAX                                     (0x0007U)

#define CSL_CMPSSA_RAMPDLYA_RESETVAL                                           (0x0000U)

/* RAMPDLYS */

#define CSL_CMPSSA_RAMPDLYS_DELAY_MASK                                         (0x1FFFU)
#define CSL_CMPSSA_RAMPDLYS_DELAY_SHIFT                                        (0x0000U)
#define CSL_CMPSSA_RAMPDLYS_DELAY_RESETVAL                                     (0x0000U)
#define CSL_CMPSSA_RAMPDLYS_DELAY_MAX                                          (0x1FFFU)

#define CSL_CMPSSA_RAMPDLYS_RESERVED_1_MASK                                    (0xE000U)
#define CSL_CMPSSA_RAMPDLYS_RESERVED_1_SHIFT                                   (0x000DU)
#define CSL_CMPSSA_RAMPDLYS_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_RAMPDLYS_RESERVED_1_MAX                                     (0x0007U)

#define CSL_CMPSSA_RAMPDLYS_RESETVAL                                           (0x0000U)

/* CTRIPLFILCTL */

#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_1_MASK                                (0x000FU)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_1_SHIFT                               (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_1_RESETVAL                            (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_1_MAX                                 (0x000FU)

#define CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_MASK                                   (0x01F0U)
#define CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_SHIFT                                  (0x0004U)
#define CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_SAMPWIN_MAX                                    (0x001FU)

#define CSL_CMPSSA_CTRIPLFILCTL_THRESH_MASK                                    (0x3E00U)
#define CSL_CMPSSA_CTRIPLFILCTL_THRESH_SHIFT                                   (0x0009U)
#define CSL_CMPSSA_CTRIPLFILCTL_THRESH_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_THRESH_MAX                                     (0x001FU)

#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_2_MASK                                (0x4000U)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_2_SHIFT                               (0x000EU)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_2_RESETVAL                            (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_RESERVED_2_MAX                                 (0x0001U)

#define CSL_CMPSSA_CTRIPLFILCTL_FILINIT_MASK                                   (0x8000U)
#define CSL_CMPSSA_CTRIPLFILCTL_FILINIT_SHIFT                                  (0x000FU)
#define CSL_CMPSSA_CTRIPLFILCTL_FILINIT_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCTL_FILINIT_MAX                                    (0x0001U)

#define CSL_CMPSSA_CTRIPLFILCTL_RESETVAL                                       (0x0000U)

/* CTRIPLFILCLKCTL */

#define CSL_CMPSSA_CTRIPLFILCLKCTL_CLKPRESCALE_MASK                            (0xFFFFU)
#define CSL_CMPSSA_CTRIPLFILCLKCTL_CLKPRESCALE_SHIFT                           (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCLKCTL_CLKPRESCALE_RESETVAL                        (0x0000U)
#define CSL_CMPSSA_CTRIPLFILCLKCTL_CLKPRESCALE_MAX                             (0xFFFFU)

#define CSL_CMPSSA_CTRIPLFILCLKCTL_RESETVAL                                    (0x0000U)

/* CTRIPHFILCTL */

#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_1_MASK                                (0x000FU)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_1_SHIFT                               (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_1_RESETVAL                            (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_1_MAX                                 (0x000FU)

#define CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_MASK                                   (0x01F0U)
#define CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_SHIFT                                  (0x0004U)
#define CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_SAMPWIN_MAX                                    (0x001FU)

#define CSL_CMPSSA_CTRIPHFILCTL_THRESH_MASK                                    (0x3E00U)
#define CSL_CMPSSA_CTRIPHFILCTL_THRESH_SHIFT                                   (0x0009U)
#define CSL_CMPSSA_CTRIPHFILCTL_THRESH_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_THRESH_MAX                                     (0x001FU)

#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_2_MASK                                (0x4000U)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_2_SHIFT                               (0x000EU)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_2_RESETVAL                            (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_RESERVED_2_MAX                                 (0x0001U)

#define CSL_CMPSSA_CTRIPHFILCTL_FILINIT_MASK                                   (0x8000U)
#define CSL_CMPSSA_CTRIPHFILCTL_FILINIT_SHIFT                                  (0x000FU)
#define CSL_CMPSSA_CTRIPHFILCTL_FILINIT_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCTL_FILINIT_MAX                                    (0x0001U)

#define CSL_CMPSSA_CTRIPHFILCTL_RESETVAL                                       (0x0000U)

/* CTRIPHFILCLKCTL */

#define CSL_CMPSSA_CTRIPHFILCLKCTL_CLKPRESCALE_MASK                            (0xFFFFU)
#define CSL_CMPSSA_CTRIPHFILCLKCTL_CLKPRESCALE_SHIFT                           (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCLKCTL_CLKPRESCALE_RESETVAL                        (0x0000U)
#define CSL_CMPSSA_CTRIPHFILCLKCTL_CLKPRESCALE_MAX                             (0xFFFFU)

#define CSL_CMPSSA_CTRIPHFILCLKCTL_RESETVAL                                    (0x0000U)

/* COMPLOCK */

#define CSL_CMPSSA_COMPLOCK_COMPCTL_MASK                                       (0x0001U)
#define CSL_CMPSSA_COMPLOCK_COMPCTL_SHIFT                                      (0x0000U)
#define CSL_CMPSSA_COMPLOCK_COMPCTL_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_COMPLOCK_COMPCTL_MAX                                        (0x0001U)

#define CSL_CMPSSA_COMPLOCK_COMPHYSCTL_MASK                                    (0x0002U)
#define CSL_CMPSSA_COMPLOCK_COMPHYSCTL_SHIFT                                   (0x0001U)
#define CSL_CMPSSA_COMPLOCK_COMPHYSCTL_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_COMPLOCK_COMPHYSCTL_MAX                                     (0x0001U)

#define CSL_CMPSSA_COMPLOCK_DACCTL_MASK                                        (0x0004U)
#define CSL_CMPSSA_COMPLOCK_DACCTL_SHIFT                                       (0x0002U)
#define CSL_CMPSSA_COMPLOCK_DACCTL_RESETVAL                                    (0x0000U)
#define CSL_CMPSSA_COMPLOCK_DACCTL_MAX                                         (0x0001U)

#define CSL_CMPSSA_COMPLOCK_CTRIP_MASK                                         (0x0008U)
#define CSL_CMPSSA_COMPLOCK_CTRIP_SHIFT                                        (0x0003U)
#define CSL_CMPSSA_COMPLOCK_CTRIP_RESETVAL                                     (0x0000U)
#define CSL_CMPSSA_COMPLOCK_CTRIP_MAX                                          (0x0001U)

#define CSL_CMPSSA_COMPLOCK_TEST_MASK                                          (0x0010U)
#define CSL_CMPSSA_COMPLOCK_TEST_SHIFT                                         (0x0004U)
#define CSL_CMPSSA_COMPLOCK_TEST_RESETVAL                                      (0x0000U)
#define CSL_CMPSSA_COMPLOCK_TEST_MAX                                           (0x0001U)

#define CSL_CMPSSA_COMPLOCK_RESERVED_1_MASK                                    (0xFFE0U)
#define CSL_CMPSSA_COMPLOCK_RESERVED_1_SHIFT                                   (0x0005U)
#define CSL_CMPSSA_COMPLOCK_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_CMPSSA_COMPLOCK_RESERVED_1_MAX                                     (0x07FFU)

#define CSL_CMPSSA_COMPLOCK_RESETVAL                                           (0x0000U)

/* DACHVALS2 */

#define CSL_CMPSSA_DACHVALS2_DACVAL_MASK                                       (0x0FFFU)
#define CSL_CMPSSA_DACHVALS2_DACVAL_SHIFT                                      (0x0000U)
#define CSL_CMPSSA_DACHVALS2_DACVAL_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_DACHVALS2_DACVAL_MAX                                        (0x0FFFU)

#define CSL_CMPSSA_DACHVALS2_RESERVED_1_MASK                                   (0xF000U)
#define CSL_CMPSSA_DACHVALS2_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_CMPSSA_DACHVALS2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_DACHVALS2_RESERVED_1_MAX                                    (0x000FU)

#define CSL_CMPSSA_DACHVALS2_RESETVAL                                          (0x0000U)

/* DACLVALS2 */

#define CSL_CMPSSA_DACLVALS2_DACVAL_MASK                                       (0x0FFFU)
#define CSL_CMPSSA_DACLVALS2_DACVAL_SHIFT                                      (0x0000U)
#define CSL_CMPSSA_DACLVALS2_DACVAL_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_DACLVALS2_DACVAL_MAX                                        (0x0FFFU)

#define CSL_CMPSSA_DACLVALS2_RESERVED_1_MASK                                   (0xF000U)
#define CSL_CMPSSA_DACLVALS2_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_CMPSSA_DACLVALS2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_CMPSSA_DACLVALS2_RESERVED_1_MAX                                    (0x000FU)

#define CSL_CMPSSA_DACLVALS2_RESETVAL                                          (0x0000U)

/* CONFIG1 */

#define CSL_CMPSSA_CONFIG1_COMPHHYS_MASK                                       (0x000FU)
#define CSL_CMPSSA_CONFIG1_COMPHHYS_SHIFT                                      (0x0000U)
#define CSL_CMPSSA_CONFIG1_COMPHHYS_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_CONFIG1_COMPHHYS_MAX                                        (0x000FU)

#define CSL_CMPSSA_CONFIG1_COMPLHYS_MASK                                       (0x00F0U)
#define CSL_CMPSSA_CONFIG1_COMPLHYS_SHIFT                                      (0x0004U)
#define CSL_CMPSSA_CONFIG1_COMPLHYS_RESETVAL                                   (0x0000U)
#define CSL_CMPSSA_CONFIG1_COMPLHYS_MAX                                        (0x000FU)

#define CSL_CMPSSA_CONFIG1_SPARE_MASK                                          (0xFF00U)
#define CSL_CMPSSA_CONFIG1_SPARE_SHIFT                                         (0x0008U)
#define CSL_CMPSSA_CONFIG1_SPARE_RESETVAL                                      (0x0000U)
#define CSL_CMPSSA_CONFIG1_SPARE_MAX                                           (0x00FFU)

#define CSL_CMPSSA_CONFIG1_RESETVAL                                            (0x0000U)

#ifdef __cplusplus
}
#endif
#endif
