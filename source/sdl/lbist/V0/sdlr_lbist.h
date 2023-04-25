/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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
#ifndef SDLR_LBIST_H_
#define SDLR_LBIST_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <sdl/sdlr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct
{
    /** LBIST control register */
    volatile uint32_t LBIST_CTRL;
    /** LBIST Pattern count */
    volatile uint32_t LBIST_PATCOUNT;
    /** LBIST Seed0 register */
    volatile uint32_t LBIST_SEED0;
    /** LBIST seed1 register */
    volatile  uint32_t LBIST_SEED1;
    /** LBIST spare0 register */
    volatile uint32_t LBIST_SPARE0;
    /** LBIST spare1 register */
    volatile  uint32_t LBIST_SPARE1;
    /** LBIST stat register */
    volatile uint32_t LBIST_STAT;
    /** LBIST Multiple Input Signature Register (MISR) */
    volatile uint32_t LBIST_MISR;
} SDL_lbistRegs;



/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_LBIST_CTRL                              (0x00000000U)
#define SDL_LBIST_PATCOUNT                          (0x00000004U)
#define SDL_LBIST_SEED0                             (0x00000008U)
#define SDL_LBIST_SEED1                             (0x0000000CU)
#define SDL_LBIST_SPARE0                            (0x00000010U)
#define SDL_LBIST_SPARE1                            (0x00000014U)
#define SDL_LBIST_STAT                              (0x00000018U)
#define SDL_LBIST_MISR                              (0x0000001CU)


/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* LBIST_CTRL */

#define SDL_LBIST_CTRL_DIVIDE_RATIO_MASK            (0x0000001FU)
#define SDL_LBIST_CTRL_DIVIDE_RATIO_SHIFT           (0x00000000U)
#define SDL_LBIST_CTRL_DIVIDE_RATIO_MAX             (0x0000001FU)

#define SDL_LBIST_CTRL_LOAD_DIV_MASK                (0x00000080U)
#define SDL_LBIST_CTRL_LOAD_DIV_SHIFT               (0x00000007U)
#define SDL_LBIST_CTRL_LOAD_DIV_MAX                 (0x00000001U)

#define SDL_LBIST_CTRL_DC_DEF_MASK                  (0x00000300U)
#define SDL_LBIST_CTRL_DC_DEF_SHIFT                 (0x00000008U)
#define SDL_LBIST_CTRL_DC_DEF_MAX                   (0x00000003U)

#define SDL_LBIST_CTRL_RUNBIST_MODE_MASK            (0x0000F000U)
#define SDL_LBIST_CTRL_RUNBIST_MODE_SHIFT           (0x0000000CU)
#define SDL_LBIST_CTRL_RUNBIST_MODE_MAX             (0x0000000FU)

#define SDL_LBIST_CTRL_BIST_RUN_MASK                (0x0F000000U)
#define SDL_LBIST_CTRL_BIST_RUN_SHIFT               (0x00000018U)
#define SDL_LBIST_CTRL_BIST_RUN_MAX                 (0x0000000FU)

#define SDL_LBIST_CTRL_BIST_RESET_MASK              (0x80000000U)
#define SDL_LBIST_CTRL_BIST_RESET_SHIFT             (0x0000001FU)
#define SDL_LBIST_CTRL_BIST_RESET_MAX               (0x00000001U)

/* LBIST_PATCOUNT */

#define SDL_LBIST_PATCOUNT_SCAN_PC_DEF_MASK         (0x0000000FU)
#define SDL_LBIST_PATCOUNT_SCAN_PC_DEF_SHIFT        (0x00000000U)
#define SDL_LBIST_PATCOUNT_SCAN_PC_DEF_MAX          (0x0000000FU)

#define SDL_LBIST_PATCOUNT_RESET_PC_DEF_MASK        (0x000000F0U)
#define SDL_LBIST_PATCOUNT_RESET_PC_DEF_SHIFT       (0x00000004U)
#define SDL_LBIST_PATCOUNT_RESET_PC_DEF_MAX         (0x0000000FU)

#define SDL_LBIST_PATCOUNT_SET_PC_DEF_MASK          (0x00000F00U)
#define SDL_LBIST_PATCOUNT_SET_PC_DEF_SHIFT         (0x00000008U)
#define SDL_LBIST_PATCOUNT_SET_PC_DEF_MAX           (0x0000000FU)

#define SDL_LBIST_PATCOUNT_STATIC_PC_DEF_MASK       (0x3FFF0000U)
#define SDL_LBIST_PATCOUNT_STATIC_PC_DEF_SHIFT      (0x00000010U)
#define SDL_LBIST_PATCOUNT_STATIC_PC_DEF_MAX        (0x00003FFFU)

/* LBIST_SEED0 */

#define SDL_LBIST_SEED0_PRPG_DEF_MASK               (0xFFFFFFFFU)
#define SDL_LBIST_SEED0_PRPG_DEF_SHIFT              (0x00000000U)
#define SDL_LBIST_SEED0_PRPG_DEF_MAX                (0xFFFFFFFFU)

/* LBIST_SEED1 */

#define SDL_LBIST_SEED1_PRPG_DEF_MASK               (0x001FFFFFU)
#define SDL_LBIST_SEED1_PRPG_DEF_SHIFT              (0x00000000U)
#define SDL_LBIST_SEED1_PRPG_DEF_MAX                (0x001FFFFFU)

/* LBIST_SPARE0 */

#define SDL_LBIST_SPARE0_LBIST_SELFTEST_EN_MASK     (0x00000001U)
#define SDL_LBIST_SPARE0_LBIST_SELFTEST_EN_SHIFT    (0x00000000U)
#define SDL_LBIST_SPARE0_LBIST_SELFTEST_EN_MAX      (0x00000001U)

#define SDL_LBIST_SPARE0_PBIST_SELFTEST_EN_MASK     (0x00000002U)
#define SDL_LBIST_SPARE0_PBIST_SELFTEST_EN_SHIFT    (0x00000001U)
#define SDL_LBIST_SPARE0_PBIST_SELFTEST_EN_MAX      (0x00000001U)

#define SDL_LBIST_SPARE0_SPARE0_MASK                (0xFFFFFFFCU)
#define SDL_LBIST_SPARE0_SPARE0_SHIFT               (0x00000002U)
#define SDL_LBIST_SPARE0_SPARE0_MAX                 (0x3FFFFFFFU)

/* LBIST_SPARE1 */

#define SDL_LBIST_SPARE1_SPARE1_MASK                (0xFFFFFFFFU)
#define SDL_LBIST_SPARE1_SPARE1_SHIFT               (0x00000000U)
#define SDL_LBIST_SPARE1_SPARE1_MAX                 (0xFFFFFFFFU)

/* LBIST_STAT */

#define SDL_LBIST_STAT_MISR_MUX_CTL_MASK            (0x000000FFU)
#define SDL_LBIST_STAT_MISR_MUX_CTL_SHIFT           (0x00000000U)
#define SDL_LBIST_STAT_MISR_MUX_CTL_MAX             (0x000000FFU)

#define SDL_LBIST_STAT_OUT_MUX_CTL_MASK             (0x00000300U)
#define SDL_LBIST_STAT_OUT_MUX_CTL_SHIFT            (0x00000008U)
#define SDL_LBIST_STAT_OUT_MUX_CTL_MAX              (0x00000003U)

#define SDL_LBIST_STAT_BIST_RUNNING_MASK            (0x00008000U)
#define SDL_LBIST_STAT_BIST_RUNNING_SHIFT           (0x0000000FU)
#define SDL_LBIST_STAT_BIST_RUNNING_MAX             (0x00000001U)

#define SDL_LBIST_STAT_BIST_DONE_MASK               (0x80000000U)
#define SDL_LBIST_STAT_BIST_DONE_SHIFT              (0x0000001FU)
#define SDL_LBIST_STAT_BIST_DONE_MAX                (0x00000001U)

/* LBIST_MISR */

#define SDL_LBIST_MISR_MISR_RESULT_MASK             (0xFFFFFFFFU)
#define SDL_LBIST_MISR_MISR_RESULT_SHIFT            (0x00000000U)
#define SDL_LBIST_MISR_MISR_RESULT_MAX              (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif /* SDLR_LBIST_H_ */
