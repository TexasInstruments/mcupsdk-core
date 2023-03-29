/********************************************************************
 * Copyright (C) Texas Instruments Incorporated 2022-2023.
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
 *  Name        : sdlr_vim.h
*/
#ifndef SDLR_VIM_H_
#define SDLR_VIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  : VIM Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RAW;                       /* Raw Status/Set Register */
    volatile uint32_t STS;                       /* Interrupt Enable Status/Clear Register */
    volatile uint32_t INTR_EN_SET;               /* Interrupt Enable Set Register */
    volatile uint32_t INTR_EN_CLR;               /* Interrupt Enabled Clear Register */
    volatile uint32_t IRQSTS;                    /* IRQ Interrupt Enable Status/Clear Register */
    volatile uint32_t FIQSTS;                    /* FIQ Interrupt Enable Status/Clear Register */
    volatile uint32_t INTMAP;                    /* Interrupt Map Register */
    volatile uint32_t INTTYPE;                   /* Interrupt Type Register */
} SDL_vimRegs_GRP;


typedef struct {
    volatile uint32_t INT;                       /* Interrupt Priority Register */
} SDL_vimRegs_PRI;


typedef struct {
    volatile uint32_t INT;                       /* Interrupt Vector Register */
} SDL_vimRegs_VEC;


typedef struct {
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t INFO;                      /* Info Register */
    volatile uint32_t PRIIRQ;                    /* Prioritized IRQ Register */
    volatile uint32_t PRIFIQ;                    /* Prioritized FIQ Register */
    volatile uint32_t IRQGSTS;                   /* IRQ Group Status Register */
    volatile uint32_t FIQGSTS;                   /* FIQ Group Status Register */
    volatile uint32_t IRQVEC;                    /* IRQ Vector Address Register */
    volatile uint32_t FIQVEC;                    /* FIQ Vector Address Register */
    volatile uint32_t ACTIRQ;                    /* ACtive IRQ Register */
    volatile uint32_t ACTFIQ;                    /* ACtive FIQ Register */
    volatile uint8_t  Resv_48[8];
    volatile uint32_t DEDVEC;                    /* DED Vector Address Register */
    volatile uint8_t  Resv_1024[972];
    SDL_vimRegs_GRP GRP[32];
    volatile uint8_t  Resv_4096[2048];
    SDL_vimRegs_PRI PRI[1024];
    SDL_vimRegs_VEC VEC[1024];
} SDL_vimRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_VIM_PID                                                            (0x00000000U)
#define SDL_VIM_INFO                                                           (0x00000004U)
#define SDL_VIM_PRIIRQ                                                         (0x00000008U)
#define SDL_VIM_PRIFIQ                                                         (0x0000000CU)
#define SDL_VIM_IRQGSTS                                                        (0x00000010U)
#define SDL_VIM_FIQGSTS                                                        (0x00000014U)
#define SDL_VIM_IRQVEC                                                         (0x00000018U)
#define SDL_VIM_FIQVEC                                                         (0x0000001CU)
#define SDL_VIM_ACTIRQ                                                         (0x00000020U)
#define SDL_VIM_ACTFIQ                                                         (0x00000024U)
#define SDL_VIM_DEDVEC                                                         (0x00000030U)
#define SDL_VIM_GRP_RAW(GRP)                                                   (0x00000400U+((GRP)*0x20U))
#define SDL_VIM_GRP_STS(GRP)                                                   (0x00000404U+((GRP)*0x20U))
#define SDL_VIM_GRP_INTR_EN_SET(GRP)                                           (0x00000408U+((GRP)*0x20U))
#define SDL_VIM_GRP_INTR_EN_CLR(GRP)                                           (0x0000040CU+((GRP)*0x20U))
#define SDL_VIM_GRP_IRQSTS(GRP)                                                (0x00000410U+((GRP)*0x20U))
#define SDL_VIM_GRP_FIQSTS(GRP)                                                (0x00000414U+((GRP)*0x20U))
#define SDL_VIM_GRP_INTMAP(GRP)                                                (0x00000418U+((GRP)*0x20U))
#define SDL_VIM_GRP_INTTYPE(GRP)                                               (0x0000041CU+((GRP)*0x20U))
#define SDL_VIM_PRI_INT(PRI)                                                   (0x00001000U+((PRI)*0x4U))
#define SDL_VIM_VEC_INT(VEC)                                                   (0x00002000U+((VEC)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RAW */

#define SDL_VIM_GRP_RAW_STS_MASK                                               (0xFFFFFFFFU)
#define SDL_VIM_GRP_RAW_STS_SHIFT                                              (0x00000000U)
#define SDL_VIM_GRP_RAW_STS_RESETVAL                                           (0x00000000U)
#define SDL_VIM_GRP_RAW_STS_MAX                                                (0xFFFFFFFFU)

#define SDL_VIM_GRP_RAW_RESETVAL                                               (0x00000000U)

/* STS */

#define SDL_VIM_GRP_STS_MSK_MASK                                               (0xFFFFFFFFU)
#define SDL_VIM_GRP_STS_MSK_SHIFT                                              (0x00000000U)
#define SDL_VIM_GRP_STS_MSK_RESETVAL                                           (0x00000000U)
#define SDL_VIM_GRP_STS_MSK_MAX                                                (0xFFFFFFFFU)

#define SDL_VIM_GRP_STS_RESETVAL                                               (0x00000000U)

/* INTR_EN_SET */

#define SDL_VIM_GRP_INTR_EN_SET_MSK_MASK                                       (0xFFFFFFFFU)
#define SDL_VIM_GRP_INTR_EN_SET_MSK_SHIFT                                      (0x00000000U)
#define SDL_VIM_GRP_INTR_EN_SET_MSK_RESETVAL                                   (0x00000000U)
#define SDL_VIM_GRP_INTR_EN_SET_MSK_MAX                                        (0xFFFFFFFFU)

#define SDL_VIM_GRP_INTR_EN_SET_RESETVAL                                       (0x00000000U)

/* INTR_EN_CLR */

#define SDL_VIM_GRP_INTR_EN_CLR_MSK_MASK                                       (0xFFFFFFFFU)
#define SDL_VIM_GRP_INTR_EN_CLR_MSK_SHIFT                                      (0x00000000U)
#define SDL_VIM_GRP_INTR_EN_CLR_MSK_RESETVAL                                   (0x00000000U)
#define SDL_VIM_GRP_INTR_EN_CLR_MSK_MAX                                        (0xFFFFFFFFU)

#define SDL_VIM_GRP_INTR_EN_CLR_RESETVAL                                       (0x00000000U)

/* IRQSTS */

#define SDL_VIM_GRP_IRQSTS_MSK_MASK                                            (0xFFFFFFFFU)
#define SDL_VIM_GRP_IRQSTS_MSK_SHIFT                                           (0x00000000U)
#define SDL_VIM_GRP_IRQSTS_MSK_RESETVAL                                        (0x00000000U)
#define SDL_VIM_GRP_IRQSTS_MSK_MAX                                             (0xFFFFFFFFU)

#define SDL_VIM_GRP_IRQSTS_RESETVAL                                            (0x00000000U)

/* FIQSTS */

#define SDL_VIM_GRP_FIQSTS_MSK_MASK                                            (0xFFFFFFFFU)
#define SDL_VIM_GRP_FIQSTS_MSK_SHIFT                                           (0x00000000U)
#define SDL_VIM_GRP_FIQSTS_MSK_RESETVAL                                        (0x00000000U)
#define SDL_VIM_GRP_FIQSTS_MSK_MAX                                             (0xFFFFFFFFU)

#define SDL_VIM_GRP_FIQSTS_RESETVAL                                            (0x00000000U)

/* INTMAP */

#define SDL_VIM_GRP_INTMAP_MSK_MASK                                            (0xFFFFFFFFU)
#define SDL_VIM_GRP_INTMAP_MSK_SHIFT                                           (0x00000000U)
#define SDL_VIM_GRP_INTMAP_MSK_RESETVAL                                        (0x00000000U)
#define SDL_VIM_GRP_INTMAP_MSK_MAX                                             (0xFFFFFFFFU)

#define SDL_VIM_GRP_INTMAP_RESETVAL                                            (0x00000000U)

/* INTTYPE */

#define SDL_VIM_GRP_INTTYPE_MSK_MASK                                           (0xFFFFFFFFU)
#define SDL_VIM_GRP_INTTYPE_MSK_SHIFT                                          (0x00000000U)
#define SDL_VIM_GRP_INTTYPE_MSK_RESETVAL                                       (0x00000000U)
#define SDL_VIM_GRP_INTTYPE_MSK_MAX                                            (0xFFFFFFFFU)

#define SDL_VIM_GRP_INTTYPE_RESETVAL                                           (0x00000000U)

/* INT */

#define SDL_VIM_PRI_INT_VAL_MASK                                               (0x0000000FU)
#define SDL_VIM_PRI_INT_VAL_SHIFT                                              (0x00000000U)
#define SDL_VIM_PRI_INT_VAL_RESETVAL                                           (0x0000000FU)
#define SDL_VIM_PRI_INT_VAL_MAX                                                (0x0000000FU)

#define SDL_VIM_PRI_INT_RESETVAL                                               (0x0000000FU)

/* INT */

#define SDL_VIM_VEC_INT_VAL_MASK                                               (0xFFFFFFFCU)
#define SDL_VIM_VEC_INT_VAL_SHIFT                                              (0x00000002U)
#define SDL_VIM_VEC_INT_VAL_RESETVAL                                           (0x00000000U)
#define SDL_VIM_VEC_INT_VAL_MAX                                                (0x3FFFFFFFU)

#define SDL_VIM_VEC_INT_RESETVAL                                               (0x00000000U)

/* PID */

#define SDL_VIM_PID_MINOR_MASK                                                 (0x0000003FU)
#define SDL_VIM_PID_MINOR_SHIFT                                                (0x00000000U)
#define SDL_VIM_PID_MINOR_RESETVAL                                             (0x00000001U)
#define SDL_VIM_PID_MINOR_MAX                                                  (0x0000003FU)

#define SDL_VIM_PID_CUSTOM_MASK                                                (0x000000C0U)
#define SDL_VIM_PID_CUSTOM_SHIFT                                               (0x00000006U)
#define SDL_VIM_PID_CUSTOM_RESETVAL                                            (0x00000000U)
#define SDL_VIM_PID_CUSTOM_MAX                                                 (0x00000003U)

#define SDL_VIM_PID_MAJOR_MASK                                                 (0x00000700U)
#define SDL_VIM_PID_MAJOR_SHIFT                                                (0x00000008U)
#define SDL_VIM_PID_MAJOR_RESETVAL                                             (0x00000000U)
#define SDL_VIM_PID_MAJOR_MAX                                                  (0x00000007U)

#define SDL_VIM_PID_RTL_MASK                                                   (0x0000F800U)
#define SDL_VIM_PID_RTL_SHIFT                                                  (0x0000000BU)
#define SDL_VIM_PID_RTL_RESETVAL                                               (0x00000000U)
#define SDL_VIM_PID_RTL_MAX                                                    (0x0000001FU)

#define SDL_VIM_PID_FUNC_MASK                                                  (0x0FFF0000U)
#define SDL_VIM_PID_FUNC_SHIFT                                                 (0x00000010U)
#define SDL_VIM_PID_FUNC_RESETVAL                                              (0x00000090U)
#define SDL_VIM_PID_FUNC_MAX                                                   (0x00000FFFU)

#define SDL_VIM_PID_BU_MASK                                                    (0x30000000U)
#define SDL_VIM_PID_BU_SHIFT                                                   (0x0000001CU)
#define SDL_VIM_PID_BU_RESETVAL                                                (0x00000002U)
#define SDL_VIM_PID_BU_MAX                                                     (0x00000003U)

#define SDL_VIM_PID_SCHEME_MASK                                                (0xC0000000U)
#define SDL_VIM_PID_SCHEME_SHIFT                                               (0x0000001EU)
#define SDL_VIM_PID_SCHEME_RESETVAL                                            (0x00000001U)
#define SDL_VIM_PID_SCHEME_MAX                                                 (0x00000003U)

#define SDL_VIM_PID_RESETVAL                                                   (0x60900001U)

/* INFO */

#define SDL_VIM_INFO_INTERRUPTS_MASK                                           (0x000007FFU)
#define SDL_VIM_INFO_INTERRUPTS_SHIFT                                          (0x00000000U)
#define SDL_VIM_INFO_INTERRUPTS_RESETVAL                                       (0x00000400U)
#define SDL_VIM_INFO_INTERRUPTS_MAX                                            (0x000007FFU)

#define SDL_VIM_INFO_RESETVAL                                                  (0x00000400U)

/* PRIIRQ */

#define SDL_VIM_PRIIRQ_VALID_MASK                                              (0x80000000U)
#define SDL_VIM_PRIIRQ_VALID_SHIFT                                             (0x0000001FU)
#define SDL_VIM_PRIIRQ_VALID_RESETVAL                                          (0x00000000U)
#define SDL_VIM_PRIIRQ_VALID_MAX                                               (0x00000001U)

#define SDL_VIM_PRIIRQ_VALID_VAL_TRUE                                          (0x1U)
#define SDL_VIM_PRIIRQ_VALID_VAL_FALSE                                         (0x0U)

#define SDL_VIM_PRIIRQ_PRI_MASK                                                (0x000F0000U)
#define SDL_VIM_PRIIRQ_PRI_SHIFT                                               (0x00000010U)
#define SDL_VIM_PRIIRQ_PRI_RESETVAL                                            (0x00000000U)
#define SDL_VIM_PRIIRQ_PRI_MAX                                                 (0x0000000FU)

#define SDL_VIM_PRIIRQ_NUM_MASK                                                (0x000003FFU)
#define SDL_VIM_PRIIRQ_NUM_SHIFT                                               (0x00000000U)
#define SDL_VIM_PRIIRQ_NUM_RESETVAL                                            (0x00000000U)
#define SDL_VIM_PRIIRQ_NUM_MAX                                                 (0x000003FFU)

#define SDL_VIM_PRIIRQ_RESETVAL                                                (0x00000000U)

/* PRIFIQ */

#define SDL_VIM_PRIFIQ_VALID_MASK                                              (0x80000000U)
#define SDL_VIM_PRIFIQ_VALID_SHIFT                                             (0x0000001FU)
#define SDL_VIM_PRIFIQ_VALID_RESETVAL                                          (0x00000000U)
#define SDL_VIM_PRIFIQ_VALID_MAX                                               (0x00000001U)

#define SDL_VIM_PRIFIQ_VALID_VAL_TRUE                                          (0x1U)
#define SDL_VIM_PRIFIQ_VALID_VAL_FALSE                                         (0x0U)

#define SDL_VIM_PRIFIQ_PRI_MASK                                                (0x000F0000U)
#define SDL_VIM_PRIFIQ_PRI_SHIFT                                               (0x00000010U)
#define SDL_VIM_PRIFIQ_PRI_RESETVAL                                            (0x00000000U)
#define SDL_VIM_PRIFIQ_PRI_MAX                                                 (0x0000000FU)

#define SDL_VIM_PRIFIQ_NUM_MASK                                                (0x000003FFU)
#define SDL_VIM_PRIFIQ_NUM_SHIFT                                               (0x00000000U)
#define SDL_VIM_PRIFIQ_NUM_RESETVAL                                            (0x00000000U)
#define SDL_VIM_PRIFIQ_NUM_MAX                                                 (0x000003FFU)

#define SDL_VIM_PRIFIQ_RESETVAL                                                (0x00000000U)

/* IRQGSTS */

#define SDL_VIM_IRQGSTS_STS_MASK                                               (0xFFFFFFFFU)
#define SDL_VIM_IRQGSTS_STS_SHIFT                                              (0x00000000U)
#define SDL_VIM_IRQGSTS_STS_RESETVAL                                           (0x00000000U)
#define SDL_VIM_IRQGSTS_STS_MAX                                                (0xFFFFFFFFU)

#define SDL_VIM_IRQGSTS_RESETVAL                                               (0x00000000U)

/* FIQGSTS */

#define SDL_VIM_FIQGSTS_STS_MASK                                               (0xFFFFFFFFU)
#define SDL_VIM_FIQGSTS_STS_SHIFT                                              (0x00000000U)
#define SDL_VIM_FIQGSTS_STS_RESETVAL                                           (0x00000000U)
#define SDL_VIM_FIQGSTS_STS_MAX                                                (0xFFFFFFFFU)

#define SDL_VIM_FIQGSTS_RESETVAL                                               (0x00000000U)

/* IRQVEC */

#define SDL_VIM_IRQVEC_ADDR_MASK                                               (0xFFFFFFFCU)
#define SDL_VIM_IRQVEC_ADDR_SHIFT                                              (0x00000002U)
#define SDL_VIM_IRQVEC_ADDR_RESETVAL                                           (0x00000000U)
#define SDL_VIM_IRQVEC_ADDR_MAX                                                (0x3FFFFFFFU)

#define SDL_VIM_IRQVEC_RESETVAL                                                (0x00000000U)

/* FIQVEC */

#define SDL_VIM_FIQVEC_ADDR_MASK                                               (0xFFFFFFFCU)
#define SDL_VIM_FIQVEC_ADDR_SHIFT                                              (0x00000002U)
#define SDL_VIM_FIQVEC_ADDR_RESETVAL                                           (0x00000000U)
#define SDL_VIM_FIQVEC_ADDR_MAX                                                (0x3FFFFFFFU)

#define SDL_VIM_FIQVEC_RESETVAL                                                (0x00000000U)

/* ACTIRQ */

#define SDL_VIM_ACTIRQ_VALID_MASK                                              (0x80000000U)
#define SDL_VIM_ACTIRQ_VALID_SHIFT                                             (0x0000001FU)
#define SDL_VIM_ACTIRQ_VALID_RESETVAL                                          (0x00000000U)
#define SDL_VIM_ACTIRQ_VALID_MAX                                               (0x00000001U)

#define SDL_VIM_ACTIRQ_VALID_VAL_TRUE                                          (0x1U)
#define SDL_VIM_ACTIRQ_VALID_VAL_FALSE                                         (0x0U)

#define SDL_VIM_ACTIRQ_PRI_MASK                                                (0x000F0000U)
#define SDL_VIM_ACTIRQ_PRI_SHIFT                                               (0x00000010U)
#define SDL_VIM_ACTIRQ_PRI_RESETVAL                                            (0x00000000U)
#define SDL_VIM_ACTIRQ_PRI_MAX                                                 (0x0000000FU)

#define SDL_VIM_ACTIRQ_NUM_MASK                                                (0x000003FFU)
#define SDL_VIM_ACTIRQ_NUM_SHIFT                                               (0x00000000U)
#define SDL_VIM_ACTIRQ_NUM_RESETVAL                                            (0x00000000U)
#define SDL_VIM_ACTIRQ_NUM_MAX                                                 (0x000003FFU)

#define SDL_VIM_ACTIRQ_RESETVAL                                                (0x00000000U)

/* ACTFIQ */

#define SDL_VIM_ACTFIQ_VALID_MASK                                              (0x80000000U)
#define SDL_VIM_ACTFIQ_VALID_SHIFT                                             (0x0000001FU)
#define SDL_VIM_ACTFIQ_VALID_RESETVAL                                          (0x00000000U)
#define SDL_VIM_ACTFIQ_VALID_MAX                                               (0x00000001U)

#define SDL_VIM_ACTFIQ_VALID_VAL_TRUE                                          (0x1U)
#define SDL_VIM_ACTFIQ_VALID_VAL_FALSE                                         (0x0U)

#define SDL_VIM_ACTFIQ_PRI_MASK                                                (0x000F0000U)
#define SDL_VIM_ACTFIQ_PRI_SHIFT                                               (0x00000010U)
#define SDL_VIM_ACTFIQ_PRI_RESETVAL                                            (0x00000000U)
#define SDL_VIM_ACTFIQ_PRI_MAX                                                 (0x0000000FU)

#define SDL_VIM_ACTFIQ_NUM_MASK                                                (0x000003FFU)
#define SDL_VIM_ACTFIQ_NUM_SHIFT                                               (0x00000000U)
#define SDL_VIM_ACTFIQ_NUM_RESETVAL                                            (0x00000000U)
#define SDL_VIM_ACTFIQ_NUM_MAX                                                 (0x000003FFU)

#define SDL_VIM_ACTFIQ_RESETVAL                                                (0x00000000U)

/* DEDVEC */

#define SDL_VIM_DEDVEC_ADDR_MASK                                               (0xFFFFFFFCU)
#define SDL_VIM_DEDVEC_ADDR_SHIFT                                              (0x00000002U)
#define SDL_VIM_DEDVEC_ADDR_RESETVAL                                           (0x00000000U)
#define SDL_VIM_DEDVEC_ADDR_MAX                                                (0x3FFFFFFFU)

#define SDL_VIM_DEDVEC_RESETVAL                                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
