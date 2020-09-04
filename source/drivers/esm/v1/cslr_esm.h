/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 */

#ifndef CSLR_ESM_H_
#define CSLR_ESM_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_ESM_ESM_REGS_BASE                                        (0x00000000U)


/**************************************************************************
* Hardware Region  : ESM Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RAW;                       /* Config Error Raw Status/Set Register */
    volatile uint32_t STS;                       /* Level Error Interrupt Enable Status/Clear Register */
    volatile uint32_t INTR_EN_SET;               /* Level Error Interrutp Enable Set Register */
    volatile uint32_t INTR_EN_CLR;               /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t INT_PRIO;                  /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t PIN_EN_SET;                /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t PIN_EN_CLR;                /* Level Error Interrupt Enabled Clear Register */
    volatile uint8_t  Resv_32[4];
} CSL_esmRegs_ERR_GRP;


typedef struct {
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t INFO;                      /* Info Register */
    volatile uint32_t EN;                        /* Global Enable Register */
    volatile uint32_t SFT_RST;                   /* Global Soft Reset Register */
    volatile uint32_t ERR_RAW;                   /* Config Error Raw Status/Set Register */
    volatile uint32_t ERR_STS;                   /* Config Error Interrupt Enable Status/Clear Register */
    volatile uint32_t ERR_EN_SET;                /* Config Error Interrutp Enable Set Register */
    volatile uint32_t ERR_EN_CLR;                /* Config Error Interrupt Enabled Clear Register */
    volatile uint32_t LOW_PRI;                   /* Low Priority Prioritized Register */
    volatile uint32_t HI_PRI;                    /* High Priority Prioritized Register */
    volatile uint32_t LOW;                       /* Low Priority Interrupt Status Register */
    volatile uint32_t HI;                        /* High Priority Interrupt Status Register */
    volatile uint32_t EOI;                       /* EOI Interrupt Register */
    volatile uint8_t  Resv_64[12];
    volatile uint32_t PIN_CTRL;                  /* Error Pin Control Register */
    volatile uint32_t PIN_STS;                   /* Error Pin Status Register */
    volatile uint32_t PIN_CNTR;                  /* Error Counter Value Register */
    volatile uint32_t PIN_CNTR_PRE;              /* Error Counter Value Pre-Load Register */
    volatile uint32_t PWMH_PIN_CNTR;             /* Error PWM High Counter Value Register */
    volatile uint32_t PWMH_PIN_CNTR_PRE;         /* Error PWM High Counter Value Pre-Load Register */
    volatile uint32_t PWML_PIN_CNTR;             /* Error PWM Low Counter Value Register */
    volatile uint32_t PWML_PIN_CNTR_PRE;         /* Error PWM Low Counter Value Pre-Load Register */
    volatile uint8_t  Resv_1024[928];
    CSL_esmRegs_ERR_GRP ERR_GRP[32];
} CSL_esmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ESM_PID                                                            (0x00000000U)
#define CSL_ESM_INFO                                                           (0x00000004U)
#define CSL_ESM_EN                                                             (0x00000008U)
#define CSL_ESM_SFT_RST                                                        (0x0000000CU)
#define CSL_ESM_ERR_RAW                                                        (0x00000010U)
#define CSL_ESM_ERR_STS                                                        (0x00000014U)
#define CSL_ESM_ERR_EN_SET                                                     (0x00000018U)
#define CSL_ESM_ERR_EN_CLR                                                     (0x0000001CU)
#define CSL_ESM_LOW_PRI                                                        (0x00000020U)
#define CSL_ESM_HI_PRI                                                         (0x00000024U)
#define CSL_ESM_LOW                                                            (0x00000028U)
#define CSL_ESM_HI                                                             (0x0000002CU)
#define CSL_ESM_EOI                                                            (0x00000030U)
#define CSL_ESM_PIN_CTRL                                                       (0x00000040U)
#define CSL_ESM_PIN_STS                                                        (0x00000044U)
#define CSL_ESM_PIN_CNTR                                                       (0x00000048U)
#define CSL_ESM_PIN_CNTR_PRE                                                   (0x0000004CU)
#define CSL_ESM_PWMH_PIN_CNTR                                                  (0x00000050U)
#define CSL_ESM_PWMH_PIN_CNTR_PRE                                              (0x00000054U)
#define CSL_ESM_PWML_PIN_CNTR                                                  (0x00000058U)
#define CSL_ESM_PWML_PIN_CNTR_PRE                                              (0x0000005CU)
#define CSL_ESM_ERR_GRP_RAW(ERR_GRP)                                           (0x00000400U+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_STS(ERR_GRP)                                           (0x00000404U+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_INTR_EN_SET(ERR_GRP)                                   (0x00000408U+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_INTR_EN_CLR(ERR_GRP)                                   (0x0000040CU+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_INT_PRIO(ERR_GRP)                                      (0x00000410U+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_PIN_EN_SET(ERR_GRP)                                    (0x00000414U+((ERR_GRP)*0x20U))
#define CSL_ESM_ERR_GRP_PIN_EN_CLR(ERR_GRP)                                    (0x00000418U+((ERR_GRP)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RAW */

#define CSL_ESM_ERR_GRP_RAW_STS_MASK                                           (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_RAW_STS_SHIFT                                          (0x00000000U)
#define CSL_ESM_ERR_GRP_RAW_STS_MAX                                            (0xFFFFFFFFU)

/* STS */

#define CSL_ESM_ERR_GRP_STS_MSK_MASK                                           (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_STS_MSK_SHIFT                                          (0x00000000U)
#define CSL_ESM_ERR_GRP_STS_MSK_MAX                                            (0xFFFFFFFFU)

/* INTR_EN_SET */

#define CSL_ESM_ERR_GRP_INTR_EN_SET_MSK_MASK                                   (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_INTR_EN_SET_MSK_SHIFT                                  (0x00000000U)
#define CSL_ESM_ERR_GRP_INTR_EN_SET_MSK_MAX                                    (0xFFFFFFFFU)

/* INTR_EN_CLR */

#define CSL_ESM_ERR_GRP_INTR_EN_CLR_MSK_MASK                                   (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_INTR_EN_CLR_MSK_SHIFT                                  (0x00000000U)
#define CSL_ESM_ERR_GRP_INTR_EN_CLR_MSK_MAX                                    (0xFFFFFFFFU)

/* INT_PRIO */

#define CSL_ESM_ERR_GRP_INT_PRIO_MSK_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_INT_PRIO_MSK_SHIFT                                     (0x00000000U)
#define CSL_ESM_ERR_GRP_INT_PRIO_MSK_MAX                                       (0xFFFFFFFFU)

/* PIN_EN_SET */

#define CSL_ESM_ERR_GRP_PIN_EN_SET_MSK_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_PIN_EN_SET_MSK_SHIFT                                   (0x00000000U)
#define CSL_ESM_ERR_GRP_PIN_EN_SET_MSK_MAX                                     (0xFFFFFFFFU)

/* PIN_EN_CLR */

#define CSL_ESM_ERR_GRP_PIN_EN_CLR_MSK_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ERR_GRP_PIN_EN_CLR_MSK_SHIFT                                   (0x00000000U)
#define CSL_ESM_ERR_GRP_PIN_EN_CLR_MSK_MAX                                     (0xFFFFFFFFU)

/* PID */

#define CSL_ESM_PID_MINOR_MASK                                                 (0x0000003FU)
#define CSL_ESM_PID_MINOR_SHIFT                                                (0x00000000U)
#define CSL_ESM_PID_MINOR_MAX                                                  (0x0000003FU)

#define CSL_ESM_PID_CUSTOM_MASK                                                (0x000000C0U)
#define CSL_ESM_PID_CUSTOM_SHIFT                                               (0x00000006U)
#define CSL_ESM_PID_CUSTOM_MAX                                                 (0x00000003U)

#define CSL_ESM_PID_MAJOR_MASK                                                 (0x00000700U)
#define CSL_ESM_PID_MAJOR_SHIFT                                                (0x00000008U)
#define CSL_ESM_PID_MAJOR_MAX                                                  (0x00000007U)

#define CSL_ESM_PID_RTL_MASK                                                   (0x0000F800U)
#define CSL_ESM_PID_RTL_SHIFT                                                  (0x0000000BU)
#define CSL_ESM_PID_RTL_MAX                                                    (0x0000001FU)

#define CSL_ESM_PID_FUNC_MASK                                                  (0x0FFF0000U)
#define CSL_ESM_PID_FUNC_SHIFT                                                 (0x00000010U)
#define CSL_ESM_PID_FUNC_MAX                                                   (0x00000FFFU)

#define CSL_ESM_PID_BU_MASK                                                    (0x30000000U)
#define CSL_ESM_PID_BU_SHIFT                                                   (0x0000001CU)
#define CSL_ESM_PID_BU_MAX                                                     (0x00000003U)

#define CSL_ESM_PID_SCHEME_MASK                                                (0xC0000000U)
#define CSL_ESM_PID_SCHEME_SHIFT                                               (0x0000001EU)
#define CSL_ESM_PID_SCHEME_MAX                                                 (0x00000003U)

/* INFO */

#define CSL_ESM_INFO_GROUPS_MASK                                               (0x000000FFU)
#define CSL_ESM_INFO_GROUPS_SHIFT                                              (0x00000000U)
#define CSL_ESM_INFO_GROUPS_MAX                                                (0x000000FFU)

#define CSL_ESM_INFO_PULSE_GROUPS_MASK                                         (0x0000FF00U)
#define CSL_ESM_INFO_PULSE_GROUPS_SHIFT                                        (0x00000008U)
#define CSL_ESM_INFO_PULSE_GROUPS_MAX                                          (0x000000FFU)

#define CSL_ESM_INFO_LAST_RESET_MASK                                           (0x80000000U)
#define CSL_ESM_INFO_LAST_RESET_SHIFT                                          (0x0000001FU)
#define CSL_ESM_INFO_LAST_RESET_MAX                                            (0x00000001U)

/* EN */

#define CSL_ESM_EN_KEY_MASK                                                    (0x0000000FU)
#define CSL_ESM_EN_KEY_SHIFT                                                   (0x00000000U)
#define CSL_ESM_EN_KEY_MAX                                                     (0x0000000FU)

/* SFT_RST */

#define CSL_ESM_SFT_RST_KEY_MASK                                               (0x0000000FU)
#define CSL_ESM_SFT_RST_KEY_SHIFT                                              (0x00000000U)
#define CSL_ESM_SFT_RST_KEY_MAX                                                (0x0000000FU)

/* ERR_RAW */

#define CSL_ESM_ERR_RAW_STS_MASK                                               (0x0000003FU)
#define CSL_ESM_ERR_RAW_STS_SHIFT                                              (0x00000000U)
#define CSL_ESM_ERR_RAW_STS_MAX                                                (0x0000003FU)

/* ERR_STS */

#define CSL_ESM_ERR_STS_MSK_MASK                                               (0x0000003FU)
#define CSL_ESM_ERR_STS_MSK_SHIFT                                              (0x00000000U)
#define CSL_ESM_ERR_STS_MSK_MAX                                                (0x0000003FU)

/* ERR_EN_SET */

#define CSL_ESM_ERR_EN_SET_MSK_MASK                                            (0x0000003FU)
#define CSL_ESM_ERR_EN_SET_MSK_SHIFT                                           (0x00000000U)
#define CSL_ESM_ERR_EN_SET_MSK_MAX                                             (0x0000003FU)

/* ERR_EN_CLR */

#define CSL_ESM_ERR_EN_CLR_MSK_MASK                                            (0x0000003FU)
#define CSL_ESM_ERR_EN_CLR_MSK_SHIFT                                           (0x00000000U)
#define CSL_ESM_ERR_EN_CLR_MSK_MAX                                             (0x0000003FU)

/* LOW_PRI */

#define CSL_ESM_LOW_PRI_PLS_MASK                                               (0xFFFF0000U)
#define CSL_ESM_LOW_PRI_PLS_SHIFT                                              (0x00000010U)
#define CSL_ESM_LOW_PRI_PLS_MAX                                                (0x0000FFFFU)

#define CSL_ESM_LOW_PRI_LVL_MASK                                               (0x0000FFFFU)
#define CSL_ESM_LOW_PRI_LVL_SHIFT                                              (0x00000000U)
#define CSL_ESM_LOW_PRI_LVL_MAX                                                (0x0000FFFFU)

/* HI_PRI */

#define CSL_ESM_HI_PRI_PLS_MASK                                                (0xFFFF0000U)
#define CSL_ESM_HI_PRI_PLS_SHIFT                                               (0x00000010U)
#define CSL_ESM_HI_PRI_PLS_MAX                                                 (0x0000FFFFU)

#define CSL_ESM_HI_PRI_LVL_MASK                                                (0x0000FFFFU)
#define CSL_ESM_HI_PRI_LVL_SHIFT                                               (0x00000000U)
#define CSL_ESM_HI_PRI_LVL_MAX                                                 (0x0000FFFFU)

/* LOW */

#define CSL_ESM_LOW_STS_MASK                                                   (0xFFFFFFFFU)
#define CSL_ESM_LOW_STS_SHIFT                                                  (0x00000000U)
#define CSL_ESM_LOW_STS_MAX                                                    (0xFFFFFFFFU)

/* HI */

#define CSL_ESM_HI_STS_MASK                                                    (0xFFFFFFFFU)
#define CSL_ESM_HI_STS_SHIFT                                                   (0x00000000U)
#define CSL_ESM_HI_STS_MAX                                                     (0xFFFFFFFFU)

/* EOI */

#define CSL_ESM_EOI_KEY_MASK                                                   (0x000007FFU)
#define CSL_ESM_EOI_KEY_SHIFT                                                  (0x00000000U)
#define CSL_ESM_EOI_KEY_MAX                                                    (0x000007FFU)

/* PIN_CTRL */

#define CSL_ESM_PIN_CTRL_PWM_EN_MASK                                           (0x000000F0U)
#define CSL_ESM_PIN_CTRL_PWM_EN_SHIFT                                          (0x00000004U)
#define CSL_ESM_PIN_CTRL_PWM_EN_MAX                                            (0x0000000FU)

#define CSL_ESM_PIN_CTRL_KEY_MASK                                              (0x0000000FU)
#define CSL_ESM_PIN_CTRL_KEY_SHIFT                                             (0x00000000U)
#define CSL_ESM_PIN_CTRL_KEY_MAX                                               (0x0000000FU)

/* PIN_STS */

#define CSL_ESM_PIN_STS_VAL_MASK                                               (0x00000001U)
#define CSL_ESM_PIN_STS_VAL_SHIFT                                              (0x00000000U)
#define CSL_ESM_PIN_STS_VAL_MAX                                                (0x00000001U)

/* PIN_CNTR */

#define CSL_ESM_PIN_CNTR_COUNT_MASK                                            (0x00FFFFFFU)
#define CSL_ESM_PIN_CNTR_COUNT_SHIFT                                           (0x00000000U)
#define CSL_ESM_PIN_CNTR_COUNT_MAX                                             (0x00FFFFFFU)

/* PIN_CNTR_PRE */

#define CSL_ESM_PIN_CNTR_PRE_COUNT_MASK                                        (0x00FFFFFFU)
#define CSL_ESM_PIN_CNTR_PRE_COUNT_SHIFT                                       (0x00000000U)
#define CSL_ESM_PIN_CNTR_PRE_COUNT_MAX                                         (0x00FFFFFFU)

/* PWMH_PIN_CNTR */

#define CSL_ESM_PWMH_PIN_CNTR_COUNT_MASK                                       (0x00FFFFFFU)
#define CSL_ESM_PWMH_PIN_CNTR_COUNT_SHIFT                                      (0x00000000U)
#define CSL_ESM_PWMH_PIN_CNTR_COUNT_MAX                                        (0x00FFFFFFU)

/* PWMH_PIN_CNTR_PRE */

#define CSL_ESM_PWMH_PIN_CNTR_PRE_COUNT_MASK                                   (0x00FFFFFFU)
#define CSL_ESM_PWMH_PIN_CNTR_PRE_COUNT_SHIFT                                  (0x00000000U)
#define CSL_ESM_PWMH_PIN_CNTR_PRE_COUNT_MAX                                    (0x00FFFFFFU)

/* PWML_PIN_CNTR */

#define CSL_ESM_PWML_PIN_CNTR_COUNT_MASK                                       (0x00FFFFFFU)
#define CSL_ESM_PWML_PIN_CNTR_COUNT_SHIFT                                      (0x00000000U)
#define CSL_ESM_PWML_PIN_CNTR_COUNT_MAX                                        (0x00FFFFFFU)

/* PWML_PIN_CNTR_PRE */

#define CSL_ESM_PWML_PIN_CNTR_PRE_COUNT_MASK                                   (0x00FFFFFFU)
#define CSL_ESM_PWML_PIN_CNTR_PRE_COUNT_SHIFT                                  (0x00000000U)
#define CSL_ESM_PWML_PIN_CNTR_PRE_COUNT_MAX                                    (0x00FFFFFFU)

#ifdef __cplusplus
}
#endif
#endif