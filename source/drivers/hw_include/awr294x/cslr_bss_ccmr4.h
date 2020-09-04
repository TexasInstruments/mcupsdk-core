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
 *  Name        : cslr_bss_ccmr4.h
*/
#ifndef CSLR_BSS_CCMR4_H_
#define CSLR_BSS_CCMR4_H_

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
    volatile uint32_t CCMSR1;
    volatile uint32_t CCMKEYR1;
    volatile uint8_t  Resv_32[24];
    volatile uint32_t CCMSR2;
    volatile uint32_t CCMKEYR2;
} CSL_bss_ccmr4Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_CCMR4_CCMSR1                                                   (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR1                                                 (0x00000004U)
#define CSL_BSS_CCMR4_CCMSR2                                                   (0x00000020U)
#define CSL_BSS_CCMR4_CCMKEYR2                                                 (0x00000024U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CCMSR1 */

#define CSL_BSS_CCMR4_CCMSR1_STE1_MASK                                         (0x00000001U)
#define CSL_BSS_CCMR4_CCMSR1_STE1_SHIFT                                        (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_STE1_RESETVAL                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_STE1_MAX                                          (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR1_STET1_MASK                                        (0x00000002U)
#define CSL_BSS_CCMR4_CCMSR1_STET1_SHIFT                                       (0x00000001U)
#define CSL_BSS_CCMR4_CCMSR1_STET1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_STET1_MAX                                         (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR1_NU0_MASK                                          (0x000000FCU)
#define CSL_BSS_CCMR4_CCMSR1_NU0_SHIFT                                         (0x00000002U)
#define CSL_BSS_CCMR4_CCMSR1_NU0_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_NU0_MAX                                           (0x0000003FU)

#define CSL_BSS_CCMR4_CCMSR1_STC1_MASK                                         (0x00000100U)
#define CSL_BSS_CCMR4_CCMSR1_STC1_SHIFT                                        (0x00000008U)
#define CSL_BSS_CCMR4_CCMSR1_STC1_RESETVAL                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_STC1_MAX                                          (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR1_NU1_MASK                                          (0x0000FE00U)
#define CSL_BSS_CCMR4_CCMSR1_NU1_SHIFT                                         (0x00000009U)
#define CSL_BSS_CCMR4_CCMSR1_NU1_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_NU1_MAX                                           (0x0000007FU)

#define CSL_BSS_CCMR4_CCMSR1_CMPE1_MASK                                        (0x00010000U)
#define CSL_BSS_CCMR4_CCMSR1_CMPE1_SHIFT                                       (0x00000010U)
#define CSL_BSS_CCMR4_CCMSR1_CMPE1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_CMPE1_MAX                                         (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR1_NU2_MASK                                          (0xFFFE0000U)
#define CSL_BSS_CCMR4_CCMSR1_NU2_SHIFT                                         (0x00000011U)
#define CSL_BSS_CCMR4_CCMSR1_NU2_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR1_NU2_MAX                                           (0x00007FFFU)

#define CSL_BSS_CCMR4_CCMSR1_RESETVAL                                          (0x00000000U)

/* CCMKEYR1 */

#define CSL_BSS_CCMR4_CCMKEYR1_MHEY1_MASK                                      (0x0000000FU)
#define CSL_BSS_CCMR4_CCMKEYR1_MHEY1_SHIFT                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR1_MHEY1_RESETVAL                                  (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR1_MHEY1_MAX                                       (0x0000000FU)

#define CSL_BSS_CCMR4_CCMKEYR1_NU3_MASK                                        (0xFFFFFFF0U)
#define CSL_BSS_CCMR4_CCMKEYR1_NU3_SHIFT                                       (0x00000004U)
#define CSL_BSS_CCMR4_CCMKEYR1_NU3_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR1_NU3_MAX                                         (0x0FFFFFFFU)

#define CSL_BSS_CCMR4_CCMKEYR1_RESETVAL                                        (0x00000000U)

/* CCMSR2 */

#define CSL_BSS_CCMR4_CCMSR2_STE2_MASK                                         (0x00000001U)
#define CSL_BSS_CCMR4_CCMSR2_STE2_SHIFT                                        (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_STE2_RESETVAL                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_STE2_MAX                                          (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR2_STET2_MASK                                        (0x00000002U)
#define CSL_BSS_CCMR4_CCMSR2_STET2_SHIFT                                       (0x00000001U)
#define CSL_BSS_CCMR4_CCMSR2_STET2_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_STET2_MAX                                         (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR2_NU4_MASK                                          (0x000000FCU)
#define CSL_BSS_CCMR4_CCMSR2_NU4_SHIFT                                         (0x00000002U)
#define CSL_BSS_CCMR4_CCMSR2_NU4_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_NU4_MAX                                           (0x0000003FU)

#define CSL_BSS_CCMR4_CCMSR2_STC2_MASK                                         (0x00000100U)
#define CSL_BSS_CCMR4_CCMSR2_STC2_SHIFT                                        (0x00000008U)
#define CSL_BSS_CCMR4_CCMSR2_STC2_RESETVAL                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_STC2_MAX                                          (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR2_NU5_MASK                                          (0x0000FE00U)
#define CSL_BSS_CCMR4_CCMSR2_NU5_SHIFT                                         (0x00000009U)
#define CSL_BSS_CCMR4_CCMSR2_NU5_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_NU5_MAX                                           (0x0000007FU)

#define CSL_BSS_CCMR4_CCMSR2_CMPE2_MASK                                        (0x00010000U)
#define CSL_BSS_CCMR4_CCMSR2_CMPE2_SHIFT                                       (0x00000010U)
#define CSL_BSS_CCMR4_CCMSR2_CMPE2_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_CMPE2_MAX                                         (0x00000001U)

#define CSL_BSS_CCMR4_CCMSR2_NU6_MASK                                          (0xFFFE0000U)
#define CSL_BSS_CCMR4_CCMSR2_NU6_SHIFT                                         (0x00000011U)
#define CSL_BSS_CCMR4_CCMSR2_NU6_RESETVAL                                      (0x00000000U)
#define CSL_BSS_CCMR4_CCMSR2_NU6_MAX                                           (0x00007FFFU)

#define CSL_BSS_CCMR4_CCMSR2_RESETVAL                                          (0x00000000U)

/* CCMKEYR2 */

#define CSL_BSS_CCMR4_CCMKEYR2_MHEY2_MASK                                      (0x0000000FU)
#define CSL_BSS_CCMR4_CCMKEYR2_MHEY2_SHIFT                                     (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR2_MHEY2_RESETVAL                                  (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR2_MHEY2_MAX                                       (0x0000000FU)

#define CSL_BSS_CCMR4_CCMKEYR2_NU7_MASK                                        (0xFFFFFFF0U)
#define CSL_BSS_CCMR4_CCMKEYR2_NU7_SHIFT                                       (0x00000004U)
#define CSL_BSS_CCMR4_CCMKEYR2_NU7_RESETVAL                                    (0x00000000U)
#define CSL_BSS_CCMR4_CCMKEYR2_NU7_MAX                                         (0x0FFFFFFFU)

#define CSL_BSS_CCMR4_CCMKEYR2_RESETVAL                                        (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
