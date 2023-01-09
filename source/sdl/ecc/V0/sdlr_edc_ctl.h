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
 *  Name        : sdlr_edc_ctl.h
*/
#ifndef SDLR_EDC_CTL_H_
#define SDLR_EDC_CTL_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <sdl/sdlr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : EDC controller serial VBUS MMRs
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t REVISION;                  /* Revision register */
    volatile uint32_t CONTROL;                   /* Control register */
    volatile uint32_t ERR_INJECT1;               /* Error inject 1 register */
    volatile uint32_t ERR_INJECT2;               /* Error inject 2 register */
    volatile uint32_t ERR_STATUS1;               /* Error status 1 register */
    volatile uint32_t ERR_STATUS2;               /* Error status 2 register */
} SDL_edc_ctlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_EDC_CTL_REVISION                                                   (0x00000010U)
#define SDL_EDC_CTL_CONTROL                                                    (0x00000014U)
#define SDL_EDC_CTL_ERR_INJECT1                                                (0x00000018U)
#define SDL_EDC_CTL_ERR_INJECT2                                                (0x0000001CU)
#define SDL_EDC_CTL_ERR_STATUS1                                                (0x00000020U)
#define SDL_EDC_CTL_ERR_STATUS2                                                (0x00000024U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define SDL_EDC_CTL_REVISION_SCHEME_MASK                                       (0xC0000000U)
#define SDL_EDC_CTL_REVISION_SCHEME_SHIFT                                      (0x0000001EU)
#define SDL_EDC_CTL_REVISION_SCHEME_RESETVAL                                   (0x00000001U)
#define SDL_EDC_CTL_REVISION_SCHEME_MAX                                        (0x00000003U)

#define SDL_EDC_CTL_REVISION_FUNC_MASK                                         (0x0FFF0000U)
#define SDL_EDC_CTL_REVISION_FUNC_SHIFT                                        (0x00000010U)
#define SDL_EDC_CTL_REVISION_FUNC_RESETVAL                                     (0x00000F40U)
#define SDL_EDC_CTL_REVISION_FUNC_MAX                                          (0x00000FFFU)

#define SDL_EDC_CTL_REVISION_RTL_MASK                                          (0x0000F800U)
#define SDL_EDC_CTL_REVISION_RTL_SHIFT                                         (0x0000000BU)
#define SDL_EDC_CTL_REVISION_RTL_RESETVAL                                      (0x00000000U)
#define SDL_EDC_CTL_REVISION_RTL_MAX                                           (0x0000001FU)

#define SDL_EDC_CTL_REVISION_MAJOR_MASK                                        (0x00000700U)
#define SDL_EDC_CTL_REVISION_MAJOR_SHIFT                                       (0x00000008U)
#define SDL_EDC_CTL_REVISION_MAJOR_RESETVAL                                    (0x00000001U)
#define SDL_EDC_CTL_REVISION_MAJOR_MAX                                         (0x00000007U)

#define SDL_EDC_CTL_REVISION_CUSTOM_MASK                                       (0x000000C0U)
#define SDL_EDC_CTL_REVISION_CUSTOM_SHIFT                                      (0x00000006U)
#define SDL_EDC_CTL_REVISION_CUSTOM_RESETVAL                                   (0x00000001U)
#define SDL_EDC_CTL_REVISION_CUSTOM_MAX                                        (0x00000003U)

#define SDL_EDC_CTL_REVISION_MINOR_MASK                                        (0x0000003FU)
#define SDL_EDC_CTL_REVISION_MINOR_SHIFT                                       (0x00000000U)
#define SDL_EDC_CTL_REVISION_MINOR_RESETVAL                                    (0x00000000U)
#define SDL_EDC_CTL_REVISION_MINOR_MAX                                         (0x0000003FU)

#define SDL_EDC_CTL_REVISION_RESETVAL                                          (0x4F400140U)

/* CONTROL */

#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_MASK                                   (0x00000F00U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_SHIFT                                  (0x00000008U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_RESETVAL                               (0x00000000U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_MAX                                    (0x0000000FU)

#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_VAL_ZEROS                              (0x0U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_VAL_F                                  (0x1U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_VAL_A                                  (0x2U)
#define SDL_EDC_CTL_CONTROL_ECC_PATTERN_VAL_FIVE                               (0x3U)

#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_MASK                                   (0x00000020U)
#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_SHIFT                                  (0x00000005U)
#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_RESETVAL                               (0x00000000U)
#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_MAX                                    (0x00000001U)

#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_VAL_KEEP_CURR_SETTINGS                 (0x0U)
#define SDL_EDC_CTL_CONTROL_FORCE_N_BIT_VAL_INC_TO_NEXT                        (0x1U)

#define SDL_EDC_CTL_CONTROL_FORCE_DE_MASK                                      (0x00000010U)
#define SDL_EDC_CTL_CONTROL_FORCE_DE_SHIFT                                     (0x00000004U)
#define SDL_EDC_CTL_CONTROL_FORCE_DE_RESETVAL                                  (0x00000000U)
#define SDL_EDC_CTL_CONTROL_FORCE_DE_MAX                                       (0x00000001U)

#define SDL_EDC_CTL_CONTROL_FORCE_SE_MASK                                      (0x00000008U)
#define SDL_EDC_CTL_CONTROL_FORCE_SE_SHIFT                                     (0x00000003U)
#define SDL_EDC_CTL_CONTROL_FORCE_SE_RESETVAL                                  (0x00000000U)
#define SDL_EDC_CTL_CONTROL_FORCE_SE_MAX                                       (0x00000001U)

#define SDL_EDC_CTL_CONTROL_ECC_CHECK_MASK                                     (0x00000002U)
#define SDL_EDC_CTL_CONTROL_ECC_CHECK_SHIFT                                    (0x00000001U)
#define SDL_EDC_CTL_CONTROL_ECC_CHECK_RESETVAL                                 (0x00000001U)
#define SDL_EDC_CTL_CONTROL_ECC_CHECK_MAX                                      (0x00000001U)

#define SDL_EDC_CTL_CONTROL_ECC_CHECK_VAL_DISABLE                              (0x0U)
#define SDL_EDC_CTL_CONTROL_ECC_CHECK_VAL_ENABLE                               (0x1U)

#define SDL_EDC_CTL_CONTROL_RESETVAL                                           (0x00000002U)

/* ERR_INJECT1 */

#define SDL_EDC_CTL_ERR_INJECT1_ECC_BIT1_MASK                                  (0x01FF0000U)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_BIT1_SHIFT                                 (0x00000010U)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_BIT1_RESETVAL                              (0x00000000U)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_BIT1_MAX                                   (0x000001FFU)

#define SDL_EDC_CTL_ERR_INJECT1_ECC_GRP_MASK                                   (0x0000FFFFU)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_GRP_SHIFT                                  (0x00000000U)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_GRP_RESETVAL                               (0x00000000U)
#define SDL_EDC_CTL_ERR_INJECT1_ECC_GRP_MAX                                    (0x0000FFFFU)

#define SDL_EDC_CTL_ERR_INJECT1_RESETVAL                                       (0x00000000U)

/* ERR_INJECT2 */

#define SDL_EDC_CTL_ERR_INJECT2_ECC_BIT2_MASK                                  (0x000001FFU)
#define SDL_EDC_CTL_ERR_INJECT2_ECC_BIT2_SHIFT                                 (0x00000000U)
#define SDL_EDC_CTL_ERR_INJECT2_ECC_BIT2_RESETVAL                              (0x00000000U)
#define SDL_EDC_CTL_ERR_INJECT2_ECC_BIT2_MAX                                   (0x000001FFU)

#define SDL_EDC_CTL_ERR_INJECT2_RESETVAL                                       (0x00000000U)

/* ERR_STATUS1 */

#define SDL_EDC_CTL_ERR_STATUS1_ERR_GRP_MASK                                   (0xFFFF0000U)
#define SDL_EDC_CTL_ERR_STATUS1_ERR_GRP_SHIFT                                  (0x00000010U)
#define SDL_EDC_CTL_ERR_STATUS1_ERR_GRP_RESETVAL                               (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_ERR_GRP_MAX                                    (0x0000FFFFU)

#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_CLR_MASK                          (0x0000C000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_CLR_SHIFT                         (0x0000000EU)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_CLR_RESETVAL                      (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_CLR_MAX                           (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_CLR_MASK                          (0x00003000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_CLR_SHIFT                         (0x0000000CU)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_CLR_RESETVAL                      (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_CLR_MAX                           (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_CLR_MASK                              (0x00000C00U)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_CLR_SHIFT                             (0x0000000AU)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_CLR_RESETVAL                          (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_CLR_MAX                               (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_CLR_MASK                              (0x00000300U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_CLR_SHIFT                             (0x00000008U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_CLR_RESETVAL                          (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_CLR_MAX                               (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_MASK                              (0x000000C0U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_SHIFT                             (0x00000006U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_RESETVAL                          (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_MAX                               (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_MASK                              (0x00000030U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_SHIFT                             (0x00000004U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_RESETVAL                          (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_INJ_COR_PEND_MAX                               (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_MASK                                  (0x0000000CU)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_SHIFT                                 (0x00000002U)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_RESETVAL                              (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_UNC_PEND_MAX                                   (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_MASK                                  (0x00000003U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_SHIFT                                 (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_RESETVAL                              (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS1_COR_PEND_MAX                                   (0x00000003U)

#define SDL_EDC_CTL_ERR_STATUS1_RESETVAL                                       (0x00000000U)

/* ERR_STATUS2 */

#define SDL_EDC_CTL_ERR_STATUS2_ERR_TYPE_MASK                                  (0xFFFF0000U)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_TYPE_SHIFT                                 (0x00000010U)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_TYPE_RESETVAL                              (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_TYPE_MAX                                   (0x0000FFFFU)

#define SDL_EDC_CTL_ERR_STATUS2_ERR_BIT_MASK                                   (0x0000FFFFU)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_BIT_SHIFT                                  (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_BIT_RESETVAL                               (0x00000000U)
#define SDL_EDC_CTL_ERR_STATUS2_ERR_BIT_MAX                                    (0x0000FFFFU)

#define SDL_EDC_CTL_ERR_STATUS2_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
