/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_icssm_intr_xbar.h
*/
#ifndef CSLR_ICSSM_INTR_XBAR_H_
#define CSLR_ICSSM_INTR_XBAR_H_

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
    volatile uint32_t PID;
    volatile uint32_t INTR_MUXCNTL[16];
} CSL_icssm_intr_xbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ICSSM_INTR_XBAR_PID                                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL(INTR_MUXCNTL)                              (0x00000004U+((INTR_MUXCNTL)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_MASK                                    (0xC0000000U)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_SHIFT                                   (0x0000001EU)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_RESETVAL                                (0x00000001U)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_MAX                                     (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_BU_MASK                                        (0x30000000U)
#define CSL_ICSSM_INTR_XBAR_PID_BU_SHIFT                                       (0x0000001CU)
#define CSL_ICSSM_INTR_XBAR_PID_BU_RESETVAL                                    (0x00000002U)
#define CSL_ICSSM_INTR_XBAR_PID_BU_MAX                                         (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_MASK                                  (0x0FFF0000U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_SHIFT                                 (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_RESETVAL                              (0x00000694U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_MAX                                   (0x00000FFFU)

#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_MASK                                    (0x0000F800U)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_SHIFT                                   (0x0000000BU)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_RESETVAL                                (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_MAX                                     (0x0000001FU)

#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_MASK                                    (0x00000700U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_SHIFT                                   (0x00000008U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_RESETVAL                                (0x00000001U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_MAX                                     (0x00000007U)

#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_MASK                                    (0x000000C0U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_SHIFT                                   (0x00000006U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_RESETVAL                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_MAX                                     (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_MINREV_MASK                                    (0x0000003FU)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_SHIFT                                   (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_RESETVAL                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_MAX                                     (0x0000003FU)

#define CSL_ICSSM_INTR_XBAR_PID_RESETVAL                                       (0x66948100U)

/* INTR_MUXCNTL */

#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_MASK                            (0x00010000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_SHIFT                           (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_MAX                             (0x00000001U)

#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_MASK                                (0x0000003FU)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_SHIFT                               (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_RESETVAL                            (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_MAX                                 (0x0000003FU)

#define CSL_ICSSM_INTR_XBAR_INTR_MUXCNTL_RESETVAL                              (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
