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

#ifndef CSLR_DAC_H_
#define CSLR_DAC_H_

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
    volatile uint16_t DACREV;
    volatile uint16_t DACCTL;
    volatile uint16_t DACVALA;
    volatile uint16_t DACVALS;
    volatile uint16_t DACOUTEN;
    volatile uint16_t DACLOCK;
    volatile uint16_t DACTRIM;
    volatile uint16_t DACCONFIG;
} CSL_dacRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_DAC_DACREV                                                         (0x00000000U)
#define CSL_DAC_DACCTL                                                         (0x00000002U)
#define CSL_DAC_DACVALA                                                        (0x00000004U)
#define CSL_DAC_DACVALS                                                        (0x00000006U)
#define CSL_DAC_DACOUTEN                                                       (0x00000008U)
#define CSL_DAC_DACLOCK                                                        (0x0000000AU)
#define CSL_DAC_DACTRIM                                                        (0x0000000CU)
#define CSL_DAC_DACCONFIG                                                      (0x0000000EU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* DACREV */

#define CSL_DAC_DACREV_REV_MASK                                                (0x00FFU)
#define CSL_DAC_DACREV_REV_SHIFT                                               (0x0000U)
#define CSL_DAC_DACREV_REV_RESETVAL                                            (0x0000U)
#define CSL_DAC_DACREV_REV_MAX                                                 (0x00FFU)

#define CSL_DAC_DACREV_RESERVED_1_MASK                                         (0xFF00U)
#define CSL_DAC_DACREV_RESERVED_1_SHIFT                                        (0x0008U)
#define CSL_DAC_DACREV_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_DAC_DACREV_RESERVED_1_MAX                                          (0x00FFU)

#define CSL_DAC_DACREV_RESETVAL                                                (0x0000U)

/* DACCTL */

#define CSL_DAC_DACCTL_DACREFSEL_MASK                                          (0x0001U)
#define CSL_DAC_DACCTL_DACREFSEL_SHIFT                                         (0x0000U)
#define CSL_DAC_DACCTL_DACREFSEL_RESETVAL                                      (0x0000U)
#define CSL_DAC_DACCTL_DACREFSEL_MAX                                           (0x0001U)

#define CSL_DAC_DACCTL_MODE_MASK                                               (0x0002U)
#define CSL_DAC_DACCTL_MODE_SHIFT                                              (0x0001U)
#define CSL_DAC_DACCTL_MODE_RESETVAL                                           (0x0000U)
#define CSL_DAC_DACCTL_MODE_MAX                                                (0x0001U)

#define CSL_DAC_DACCTL_LOADMODE_MASK                                           (0x0004U)
#define CSL_DAC_DACCTL_LOADMODE_SHIFT                                          (0x0002U)
#define CSL_DAC_DACCTL_LOADMODE_RESETVAL                                       (0x0000U)
#define CSL_DAC_DACCTL_LOADMODE_MAX                                            (0x0001U)

#define CSL_DAC_DACCTL_RESERVED_1_MASK                                         (0x0008U)
#define CSL_DAC_DACCTL_RESERVED_1_SHIFT                                        (0x0003U)
#define CSL_DAC_DACCTL_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_DAC_DACCTL_RESERVED_1_MAX                                          (0x0001U)

#define CSL_DAC_DACCTL_SYNCSEL_MASK                                            (0x01F0U)
#define CSL_DAC_DACCTL_SYNCSEL_SHIFT                                           (0x0004U)
#define CSL_DAC_DACCTL_SYNCSEL_RESETVAL                                        (0x0000U)
#define CSL_DAC_DACCTL_SYNCSEL_MAX                                             (0x001FU)

#define CSL_DAC_DACCTL_RESERVED_2_MASK                                         (0xFE00U)
#define CSL_DAC_DACCTL_RESERVED_2_SHIFT                                        (0x0009U)
#define CSL_DAC_DACCTL_RESERVED_2_RESETVAL                                     (0x0000U)
#define CSL_DAC_DACCTL_RESERVED_2_MAX                                          (0x007FU)

#define CSL_DAC_DACCTL_RESETVAL                                                (0x0000U)

/* DACVALA */

#define CSL_DAC_DACVALA_DACVALA_MASK                                           (0x0FFFU)
#define CSL_DAC_DACVALA_DACVALA_SHIFT                                          (0x0000U)
#define CSL_DAC_DACVALA_DACVALA_RESETVAL                                       (0x0000U)
#define CSL_DAC_DACVALA_DACVALA_MAX                                            (0x0FFFU)

#define CSL_DAC_DACVALA_RESERVED_1_MASK                                        (0xF000U)
#define CSL_DAC_DACVALA_RESERVED_1_SHIFT                                       (0x000CU)
#define CSL_DAC_DACVALA_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_DAC_DACVALA_RESERVED_1_MAX                                         (0x000FU)

#define CSL_DAC_DACVALA_RESETVAL                                               (0x0000U)

/* DACVALS */

#define CSL_DAC_DACVALS_DACVALS_MASK                                           (0x0FFFU)
#define CSL_DAC_DACVALS_DACVALS_SHIFT                                          (0x0000U)
#define CSL_DAC_DACVALS_DACVALS_RESETVAL                                       (0x0000U)
#define CSL_DAC_DACVALS_DACVALS_MAX                                            (0x0FFFU)

#define CSL_DAC_DACVALS_RESERVED_1_MASK                                        (0xF000U)
#define CSL_DAC_DACVALS_RESERVED_1_SHIFT                                       (0x000CU)
#define CSL_DAC_DACVALS_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_DAC_DACVALS_RESERVED_1_MAX                                         (0x000FU)

#define CSL_DAC_DACVALS_RESETVAL                                               (0x0000U)

/* DACOUTEN */

#define CSL_DAC_DACOUTEN_DACOUTEN_MASK                                         (0x0001U)
#define CSL_DAC_DACOUTEN_DACOUTEN_SHIFT                                        (0x0000U)
#define CSL_DAC_DACOUTEN_DACOUTEN_RESETVAL                                     (0x0000U)
#define CSL_DAC_DACOUTEN_DACOUTEN_MAX                                          (0x0001U)

#define CSL_DAC_DACOUTEN_RESERVED_1_MASK                                       (0xFFFEU)
#define CSL_DAC_DACOUTEN_RESERVED_1_SHIFT                                      (0x0001U)
#define CSL_DAC_DACOUTEN_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_DAC_DACOUTEN_RESERVED_1_MAX                                        (0x7FFFU)

#define CSL_DAC_DACOUTEN_RESETVAL                                              (0x0000U)

/* DACLOCK */

#define CSL_DAC_DACLOCK_DACCTL_MASK                                            (0x0001U)
#define CSL_DAC_DACLOCK_DACCTL_SHIFT                                           (0x0000U)
#define CSL_DAC_DACLOCK_DACCTL_RESETVAL                                        (0x0000U)
#define CSL_DAC_DACLOCK_DACCTL_MAX                                             (0x0001U)

#define CSL_DAC_DACLOCK_DACVAL_MASK                                            (0x0002U)
#define CSL_DAC_DACLOCK_DACVAL_SHIFT                                           (0x0001U)
#define CSL_DAC_DACLOCK_DACVAL_RESETVAL                                        (0x0000U)
#define CSL_DAC_DACLOCK_DACVAL_MAX                                             (0x0001U)

#define CSL_DAC_DACLOCK_DACOUTEN_MASK                                          (0x0004U)
#define CSL_DAC_DACLOCK_DACOUTEN_SHIFT                                         (0x0002U)
#define CSL_DAC_DACLOCK_DACOUTEN_RESETVAL                                      (0x0000U)
#define CSL_DAC_DACLOCK_DACOUTEN_MAX                                           (0x0001U)

#define CSL_DAC_DACLOCK_RESERVED_1_MASK                                        (0x0FF8U)
#define CSL_DAC_DACLOCK_RESERVED_1_SHIFT                                       (0x0003U)
#define CSL_DAC_DACLOCK_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_DAC_DACLOCK_RESERVED_1_MAX                                         (0x01FFU)

#define CSL_DAC_DACLOCK_KEY_MASK                                               (0xF000U)
#define CSL_DAC_DACLOCK_KEY_SHIFT                                              (0x000CU)
#define CSL_DAC_DACLOCK_KEY_RESETVAL                                           (0x0000U)
#define CSL_DAC_DACLOCK_KEY_MAX                                                (0x000FU)

#define CSL_DAC_DACLOCK_RESETVAL                                               (0x0000U)

/* DACTRIM */

#define CSL_DAC_DACTRIM_OFFSET_TRIM_MASK                                       (0x00FFU)
#define CSL_DAC_DACTRIM_OFFSET_TRIM_SHIFT                                      (0x0000U)
#define CSL_DAC_DACTRIM_OFFSET_TRIM_RESETVAL                                   (0x0000U)
#define CSL_DAC_DACTRIM_OFFSET_TRIM_MAX                                        (0x00FFU)

#define CSL_DAC_DACTRIM_GAIN_TRIM_MASK                                         (0x0F00U)
#define CSL_DAC_DACTRIM_GAIN_TRIM_SHIFT                                        (0x0008U)
#define CSL_DAC_DACTRIM_GAIN_TRIM_RESETVAL                                     (0x0000U)
#define CSL_DAC_DACTRIM_GAIN_TRIM_MAX                                          (0x000FU)

#define CSL_DAC_DACTRIM_RESERVED_1_MASK                                        (0xF000U)
#define CSL_DAC_DACTRIM_RESERVED_1_SHIFT                                       (0x000CU)
#define CSL_DAC_DACTRIM_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_DAC_DACTRIM_RESERVED_1_MAX                                         (0x000FU)

#define CSL_DAC_DACTRIM_RESETVAL                                               (0x0000U)

/* DACCONFIG */

#define CSL_DAC_DACCONFIG_CONFIG_MASK                                          (0xFFFFU)
#define CSL_DAC_DACCONFIG_CONFIG_SHIFT                                         (0x0000U)
#define CSL_DAC_DACCONFIG_CONFIG_RESETVAL                                      (0x0000U)
#define CSL_DAC_DACCONFIG_CONFIG_MAX                                           (0xFFFFU)

#define CSL_DAC_DACCONFIG_RESETVAL                                             (0x0000U)

#ifdef __cplusplus
}
#endif
#endif
