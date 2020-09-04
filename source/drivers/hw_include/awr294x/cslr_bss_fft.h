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
 *  Name        : cslr_bss_fft.h
*/
#ifndef CSLR_BSS_FFT_H_
#define CSLR_BSS_FFT_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t BIST_FFT_REG0;
    volatile uint32_t BIST_FFT_REG1;
    volatile uint32_t BIST_FFT_REG2;
    volatile uint32_t BIST_FFT_REG3;
    volatile uint32_t BIST_FFT_REG4;
    volatile uint32_t BIST_FFT_REG5;
    volatile uint32_t BIST_FFT_REG6;
    volatile uint32_t BIST_FFT_REG7;
} CSL_bss_fftRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_FFT_BIST_FFT_REG0                                              (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1                                              (0x00000004U)
#define CSL_BSS_FFT_BIST_FFT_REG2                                              (0x00000008U)
#define CSL_BSS_FFT_BIST_FFT_REG3                                              (0x0000000CU)
#define CSL_BSS_FFT_BIST_FFT_REG4                                              (0x00000010U)
#define CSL_BSS_FFT_BIST_FFT_REG5                                              (0x00000014U)
#define CSL_BSS_FFT_BIST_FFT_REG6                                              (0x00000018U)
#define CSL_BSS_FFT_BIST_FFT_REG7                                              (0x0000001CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* BIST_FFT_REG0 */

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_ENABLE_MODE_MASK                         (0x00000003U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_ENABLE_MODE_SHIFT                        (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_ENABLE_MODE_RESETVAL                     (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_ENABLE_MODE_MAX                          (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG0_NU1_MASK                                     (0x000000FCU)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU1_SHIFT                                    (0x00000002U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU1_MAX                                      (0x0000003FU)

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_CLK_EN_MASK                              (0x00000100U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_CLK_EN_SHIFT                             (0x00000008U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_CLK_EN_RESETVAL                          (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_CLK_EN_MAX                               (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG0_NU2_MASK                                     (0x0000FE00U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU2_MAX                                      (0x0000007FU)

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_SIZE_MASK                                (0x00070000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_SIZE_SHIFT                               (0x00000010U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_SIZE_RESETVAL                            (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_SIZE_MAX                                 (0x00000007U)

#define CSL_BSS_FFT_BIST_FFT_REG0_NU3_MASK                                     (0x00F80000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU3_SHIFT                                    (0x00000013U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU3_MAX                                      (0x0000001FU)

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_2K_SEL_MASK                              (0x03000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_2K_SEL_SHIFT                             (0x00000018U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_2K_SEL_RESETVAL                          (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_2K_SEL_MAX                               (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG0_NU4_MASK                                     (0x0C000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU4_SHIFT                                    (0x0000001AU)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU4_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_NU4_MAX                                      (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL1_MASK                             (0x30000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL1_SHIFT                            (0x0000001CU)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL1_RESETVAL                         (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL1_MAX                              (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL2_MASK                             (0xC0000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL2_SHIFT                            (0x0000001EU)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL2_RESETVAL                         (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG0_FFT_1K_SEL2_MAX                              (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG0_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG1 */

#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_FRAC_SCALE_MASK                          (0x0000007FU)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_FRAC_SCALE_SHIFT                         (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_FRAC_SCALE_RESETVAL                      (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_FRAC_SCALE_MAX                           (0x0000007FU)

#define CSL_BSS_FFT_BIST_FFT_REG1_NU1_MASK                                     (0x00000080U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU1_SHIFT                                    (0x00000007U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU1_MAX                                      (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_INT_SCALE_MASK                           (0x00000F00U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_INT_SCALE_SHIFT                          (0x00000008U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_INT_SCALE_RESETVAL                       (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_INT_SCALE_MAX                            (0x0000000FU)

#define CSL_BSS_FFT_BIST_FFT_REG1_NU2_MASK                                     (0x0000F000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU2_SHIFT                                    (0x0000000CU)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU2_MAX                                      (0x0000000FU)

#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_NACC_MASK                                (0x0FFF0000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_NACC_SHIFT                               (0x00000010U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_NACC_RESETVAL                            (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_FFT_NACC_MAX                                 (0x00000FFFU)

#define CSL_BSS_FFT_BIST_FFT_REG1_NU3_MASK                                     (0xF0000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU3_SHIFT                                    (0x0000001CU)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG1_NU3_MAX                                      (0x0000000FU)

#define CSL_BSS_FFT_BIST_FFT_REG1_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG2 */

#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_REAL_MASK                                (0x00000001U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_REAL_SHIFT                               (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_REAL_RESETVAL                            (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_REAL_MAX                                 (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG2_WINDOW_BYPASS_MASK                           (0x00000002U)
#define CSL_BSS_FFT_BIST_FFT_REG2_WINDOW_BYPASS_SHIFT                          (0x00000001U)
#define CSL_BSS_FFT_BIST_FFT_REG2_WINDOW_BYPASS_RESETVAL                       (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_WINDOW_BYPASS_MAX                            (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG2_NU1_MASK                                     (0x000000FCU)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU1_SHIFT                                    (0x00000002U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU1_MAX                                      (0x0000003FU)

#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_ACCUM_CONT_MASK                          (0x00000100U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_ACCUM_CONT_SHIFT                         (0x00000008U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_ACCUM_CONT_RESETVAL                      (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_ACCUM_CONT_MAX                           (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG2_NU2_MASK                                     (0x0000FE00U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU2_MAX                                      (0x0000007FU)

#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_MEM_ACCESS_MODE_MASK                     (0x00030000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_MEM_ACCESS_MODE_SHIFT                    (0x00000010U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_MEM_ACCESS_MODE_RESETVAL                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_FFT_MEM_ACCESS_MODE_MAX                      (0x00000003U)

#define CSL_BSS_FFT_BIST_FFT_REG2_NU3_MASK                                     (0xFFFC0000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU3_SHIFT                                    (0x00000012U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG2_NU3_MAX                                      (0x00003FFFU)

#define CSL_BSS_FFT_BIST_FFT_REG2_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG3 */

#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_MASK                           (0x00000001U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_SHIFT                          (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_RESETVAL                       (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_MAX                            (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG3_NU1_MASK                                     (0x000000FEU)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU1_SHIFT                                    (0x00000001U)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU1_MAX                                      (0x0000007FU)

#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_STATUS_MASK                    (0x00000100U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_STATUS_SHIFT                   (0x00000008U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_STATUS_RESETVAL                (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG3_INIT_FFT_IRAM_STATUS_MAX                     (0x00000001U)

#define CSL_BSS_FFT_BIST_FFT_REG3_NU2_MASK                                     (0xFFFFFE00U)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG3_NU2_MAX                                      (0x007FFFFFU)

#define CSL_BSS_FFT_BIST_FFT_REG3_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG4 */

#define CSL_BSS_FFT_BIST_FFT_REG4_SPARE1_WR_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_FFT_BIST_FFT_REG4_SPARE1_WR_SHIFT                              (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG4_SPARE1_WR_RESETVAL                           (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG4_SPARE1_WR_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_FFT_BIST_FFT_REG4_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG5 */

#define CSL_BSS_FFT_BIST_FFT_REG5_SPARE2_WR_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_FFT_BIST_FFT_REG5_SPARE2_WR_SHIFT                              (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG5_SPARE2_WR_RESETVAL                           (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG5_SPARE2_WR_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_FFT_BIST_FFT_REG5_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG6 */

#define CSL_BSS_FFT_BIST_FFT_REG6_SPARE3_RD_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_FFT_BIST_FFT_REG6_SPARE3_RD_SHIFT                              (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG6_SPARE3_RD_RESETVAL                           (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG6_SPARE3_RD_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_FFT_BIST_FFT_REG6_RESETVAL                                     (0x00000000U)

/* BIST_FFT_REG7 */

#define CSL_BSS_FFT_BIST_FFT_REG7_SPARE4_RD_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_FFT_BIST_FFT_REG7_SPARE4_RD_SHIFT                              (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG7_SPARE4_RD_RESETVAL                           (0x00000000U)
#define CSL_BSS_FFT_BIST_FFT_REG7_SPARE4_RD_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_FFT_BIST_FFT_REG7_RESETVAL                                     (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
