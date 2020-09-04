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
 *  Name        : cslr_bss_vim_parity.h
*/
#ifndef CSLR_BSS_VIM_PARITY_H_
#define CSLR_BSS_VIM_PARITY_H_

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
    volatile uint8_t  Resv_236[236];
    volatile uint32_t PARFLG;
    volatile uint32_t PARCTL;
    volatile uint32_t ADDERR;
    volatile uint32_t FBPAERR;
    volatile uint32_t SBERR;
} CSL_bss_vim_parityRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_VIM_PARITY_PARFLG                                              (0x000000ECU)
#define CSL_BSS_VIM_PARITY_PARCTL                                              (0x000000F0U)
#define CSL_BSS_VIM_PARITY_ADDERR                                              (0x000000F4U)
#define CSL_BSS_VIM_PARITY_FBPAERR                                             (0x000000F8U)
#define CSL_BSS_VIM_PARITY_SBERR                                               (0x000000FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PARFLG */

#define CSL_BSS_VIM_PARITY_PARFLG_PARFLG_MASK                                  (0x00000001U)
#define CSL_BSS_VIM_PARITY_PARFLG_PARFLG_SHIFT                                 (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARFLG_PARFLG_RESETVAL                              (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARFLG_PARFLG_MAX                                   (0x00000001U)

#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED0_MASK                               (0x000000FEU)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED0_SHIFT                              (0x00000001U)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED0_MAX                                (0x0000007FU)

#define CSL_BSS_VIM_PARITY_PARFLG_SBERR_MASK                                   (0x00000100U)
#define CSL_BSS_VIM_PARITY_PARFLG_SBERR_SHIFT                                  (0x00000008U)
#define CSL_BSS_VIM_PARITY_PARFLG_SBERR_RESETVAL                               (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARFLG_SBERR_MAX                                    (0x00000001U)

#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED1_MASK                               (0xFFFFFE00U)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED1_SHIFT                              (0x00000009U)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARFLG_RESERVED1_MAX                                (0x007FFFFFU)

#define CSL_BSS_VIM_PARITY_PARFLG_RESETVAL                                     (0x00000000U)

/* PARCTL */

#define CSL_BSS_VIM_PARITY_PARCTL_PARENA_MASK                                  (0x0000000FU)
#define CSL_BSS_VIM_PARITY_PARCTL_PARENA_SHIFT                                 (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_PARENA_RESETVAL                              (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_PARENA_MAX                                   (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED0_MASK                               (0x000000F0U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED0_SHIFT                              (0x00000004U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED0_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED0_MAX                                (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_TEST_MASK                                    (0x00000F00U)
#define CSL_BSS_VIM_PARITY_PARCTL_TEST_SHIFT                                   (0x00000008U)
#define CSL_BSS_VIM_PARITY_PARCTL_TEST_RESETVAL                                (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_TEST_MAX                                     (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED1_MASK                               (0x0000F000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED1_SHIFT                              (0x0000000CU)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED1_MAX                                (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_EDAC_MASK                                    (0x000F0000U)
#define CSL_BSS_VIM_PARITY_PARCTL_EDAC_SHIFT                                   (0x00000010U)
#define CSL_BSS_VIM_PARITY_PARCTL_EDAC_RESETVAL                                (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_EDAC_MAX                                     (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED2_MASK                               (0x00F00000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED2_SHIFT                              (0x00000014U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED2_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED2_MAX                                (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_SBE_MASK                                     (0x0F000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_SBE_SHIFT                                    (0x00000018U)
#define CSL_BSS_VIM_PARITY_PARCTL_SBE_RESETVAL                                 (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_SBE_MAX                                      (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED3_MASK                               (0xF0000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED3_SHIFT                              (0x0000001CU)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED3_RESETVAL                           (0x00000000U)
#define CSL_BSS_VIM_PARITY_PARCTL_RESERVED3_MAX                                (0x0000000FU)

#define CSL_BSS_VIM_PARITY_PARCTL_RESETVAL                                     (0x00000000U)

/* ADDERR */

#define CSL_BSS_VIM_PARITY_ADDERR_WORD_OFFSET_MASK                             (0x00000003U)
#define CSL_BSS_VIM_PARITY_ADDERR_WORD_OFFSET_SHIFT                            (0x00000000U)
#define CSL_BSS_VIM_PARITY_ADDERR_WORD_OFFSET_RESETVAL                         (0x00000000U)
#define CSL_BSS_VIM_PARITY_ADDERR_WORD_OFFSET_MAX                              (0x00000003U)

#define CSL_BSS_VIM_PARITY_ADDERR_ADDERR_MASK                                  (0x000003FCU)
#define CSL_BSS_VIM_PARITY_ADDERR_ADDERR_SHIFT                                 (0x00000002U)
#define CSL_BSS_VIM_PARITY_ADDERR_ADDERR_RESETVAL                              (0x00000000U)
#define CSL_BSS_VIM_PARITY_ADDERR_ADDERR_MAX                                   (0x000000FFU)

#define CSL_BSS_VIM_PARITY_ADDERR_RAMBASE_MASK                                 (0xFFFFFC00U)
#define CSL_BSS_VIM_PARITY_ADDERR_RAMBASE_SHIFT                                (0x0000000AU)
#define CSL_BSS_VIM_PARITY_ADDERR_RAMBASE_RESETVAL                             (0x00000000U)
#define CSL_BSS_VIM_PARITY_ADDERR_RAMBASE_MAX                                  (0x003FFFFFU)

#define CSL_BSS_VIM_PARITY_ADDERR_RESETVAL                                     (0x00000000U)

/* FBPAERR */

#define CSL_BSS_VIM_PARITY_FBPAERR_FBPAERR_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_VIM_PARITY_FBPAERR_FBPAERR_SHIFT                               (0x00000000U)
#define CSL_BSS_VIM_PARITY_FBPAERR_FBPAERR_RESETVAL                            (0x00000000U)
#define CSL_BSS_VIM_PARITY_FBPAERR_FBPAERR_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_VIM_PARITY_FBPAERR_RESETVAL                                    (0x00000000U)

/* SBERR */

#define CSL_BSS_VIM_PARITY_SBERR_WORD_OFFSET_MASK                              (0x00000003U)
#define CSL_BSS_VIM_PARITY_SBERR_WORD_OFFSET_SHIFT                             (0x00000000U)
#define CSL_BSS_VIM_PARITY_SBERR_WORD_OFFSET_RESETVAL                          (0x00000000U)
#define CSL_BSS_VIM_PARITY_SBERR_WORD_OFFSET_MAX                               (0x00000003U)

#define CSL_BSS_VIM_PARITY_SBERR_SBERR_MASK                                    (0x000003FCU)
#define CSL_BSS_VIM_PARITY_SBERR_SBERR_SHIFT                                   (0x00000002U)
#define CSL_BSS_VIM_PARITY_SBERR_SBERR_RESETVAL                                (0x00000000U)
#define CSL_BSS_VIM_PARITY_SBERR_SBERR_MAX                                     (0x000000FFU)

#define CSL_BSS_VIM_PARITY_SBERR_RAMBASE_MASK                                  (0xFFFFFC00U)
#define CSL_BSS_VIM_PARITY_SBERR_RAMBASE_SHIFT                                 (0x0000000AU)
#define CSL_BSS_VIM_PARITY_SBERR_RAMBASE_RESETVAL                              (0x00000000U)
#define CSL_BSS_VIM_PARITY_SBERR_RAMBASE_MAX                                   (0x003FFFFFU)

#define CSL_BSS_VIM_PARITY_SBERR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
