/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_rl2_of_r5fss0_core1.h
*/
#ifndef CSLR_RL2_OF_R5FSS0_CORE1_H_
#define CSLR_RL2_OF_R5FSS0_CORE1_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_RL2_OF_R5FSS0_CORE1_REGS_BASE                                                   (0x00000000U)


/**************************************************************************
* Hardware Region  : rl2_of_r5fss0_core1 registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t ADR;                       /* REMote 'n' Address LSW */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t LEN;                       /* REMote 'n' length */
    volatile uint8_t  Resv_16[4];
} CSL_rl2_of_r5fss0_core1Regs_rem;


typedef struct {
    volatile uint32_t LO;                        /* FLC Lo address 'n' */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t HI;                        /* FLC High address 'n' */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t RA;                        /* FLC Remote Address 'n' */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t CTL;                       /* FLC COntrol 'n' */
    volatile uint8_t  Resv_32[4];
} CSL_rl2_of_r5fss0_core1Regs_flc;


typedef struct {
    volatile uint32_t CTL;                       /* RAT Region Control 'n' Register */
    volatile uint32_t RBA;                       /* RAT Region Base Address 'n' Register */
    volatile uint32_t RTA;                       /* RAT Region Translated Address 0 */
    volatile uint8_t  Resv_16[4];
} CSL_rl2_of_r5fss0_core1Regs_rat;


typedef struct {
    volatile uint32_t MOD_VER;                   /* Module and Version */
    volatile uint32_t L2_CTRL;                   /* Control Register */
    volatile uint32_t L2_STS;                    /* Status */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t L2_LO;                     /* L2 Lo address LSW */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t L2_HI;                     /* L2 Hi address LSW */
    volatile uint8_t  Resv_32[4];
    CSL_rl2_of_r5fss0_core1Regs_rem REM[3];
    volatile uint8_t  Resv_120[40];
    volatile uint32_t L2HC;                      /* L2 HIT Counter register */
    volatile uint32_t L2MC;                      /* L2 MISS Counter register */
    volatile uint32_t IRQSTATUS_RAW;             /* Interrupt Raw Status Register */
    volatile uint32_t IRQSTATUS_MSK;             /* Interrupt Masked Status Register */
    volatile uint32_t IRQENABLE_SET;             /* Interrupt Enable Set Register */
    volatile uint32_t IRQENABLE_CLR;             /* Interrupt Enable Clear Register */
    volatile uint8_t  Resv_260[116];
    volatile uint32_t FLC_CFG;                   /* FLC Config Register */
    volatile uint32_t FLC_STS;                   /* FLC Status Register */
    volatile uint8_t  Resv_272[4];
    volatile uint32_t FLC_DBG0;                  /* FLC Debug 0 Register */
    volatile uint32_t FLC_DBG1;                  /* FLC Debug 1 Register */
    volatile uint8_t  Resv_288[8];
    CSL_rl2_of_r5fss0_core1Regs_flc FLC[4];
    volatile uint8_t  Resv_516[100];
    volatile uint32_t RAT_CFG;                   /* RAT Config Register */
    volatile uint8_t  Resv_544[24];
    CSL_rl2_of_r5fss0_core1Regs_rat RAT[4];
} CSL_rl2_of_r5fss0_core1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER                                                     (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL                                                     (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS                                                      (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO                                                       (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI                                                       (0x00000018U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR(REM)                                                (0x00000020U+((REM)*0x10U))
#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN(REM)                                                (0x00000028U+((REM)*0x10U))
#define CSL_RL2_OF_R5FSS0_CORE1_L2HC                                                        (0x00000078U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2MC                                                        (0x0000007CU)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW                                               (0x00000080U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK                                               (0x00000084U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET                                               (0x00000088U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR                                               (0x0000008CU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG                                                     (0x00000104U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS                                                     (0x00000108U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0                                                    (0x00000110U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1                                                    (0x00000114U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO(FLC)                                                 (0x00000120U+((FLC)*0x20U))
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI(FLC)                                                 (0x00000128U+((FLC)*0x20U))
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA(FLC)                                                 (0x00000130U+((FLC)*0x20U))
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL(FLC)                                                (0x00000138U+((FLC)*0x20U))
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG                                                     (0x00000204U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL(RAT)                                                (0x00000220U+((RAT)*0x10U))
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA(RAT)                                                (0x00000224U+((RAT)*0x10U))
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA(RAT)                                                (0x00000228U+((RAT)*0x10U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ADR */

#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR_ADR_LSW_MASK                                        (0xFFFFF800U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR_ADR_LSW_SHIFT                                       (0x0000000BU)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR_ADR_LSW_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR_ADR_LSW_MAX                                         (0x001FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_REM_ADR_RESETVAL                                            (0x00000000U)

/* LEN */

#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN_LEN_MASK                                            (0x0007FFC0U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN_LEN_SHIFT                                           (0x00000006U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN_LEN_RESETVAL                                        (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN_LEN_MAX                                             (0x00001FFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_REM_LEN_RESETVAL                                            (0x00000000U)

/* LO */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO_ADR_LO_MASK                                          (0xFFFFF000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO_ADR_LO_SHIFT                                         (0x0000000CU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO_ADR_LO_RESETVAL                                      (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO_ADR_LO_MAX                                           (0x000FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_LO_RESETVAL                                             (0x00000000U)

/* HI */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI_ADR_HI_MASK                                          (0xFFFFF000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI_ADR_HI_SHIFT                                         (0x0000000CU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI_ADR_HI_RESETVAL                                      (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI_ADR_HI_MAX                                           (0x000FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_HI_RESETVAL                                             (0x00000000U)

/* RA */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA_RADR_MASK                                            (0xFFFFF000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA_RADR_SHIFT                                           (0x0000000CU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA_RADR_RESETVAL                                        (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA_RADR_MAX                                             (0x000FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_RA_RESETVAL                                             (0x00000000U)

/* CTL */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FENABLE_MASK                                        (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FENABLE_SHIFT                                       (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FENABLE_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FENABLE_MAX                                         (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FCOPIED_MASK                                        (0x10000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FCOPIED_SHIFT                                       (0x0000001CU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FCOPIED_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_FCOPIED_MAX                                         (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CTL_RESETVAL                                            (0x00000000U)

/* CTL */

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_REN_MASK                                            (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_REN_SHIFT                                           (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_REN_RESETVAL                                        (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_REN_MAX                                             (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_SIZE_MASK                                           (0x0000003FU)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_SIZE_SHIFT                                          (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_SIZE_RESETVAL                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_SIZE_MAX                                            (0x0000003FU)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CTL_RESETVAL                                            (0x00000000U)

/* RBA */

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA_BASE_MASK                                           (0xFFFFFFFFU)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA_BASE_SHIFT                                          (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA_BASE_RESETVAL                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA_BASE_MAX                                            (0xFFFFFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RBA_RESETVAL                                            (0x00000000U)

/* RTA */

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA_TRANS_MASK                                          (0xFFFFFFFFU)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA_TRANS_SHIFT                                         (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA_TRANS_RESETVAL                                      (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA_TRANS_MAX                                           (0xFFFFFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_RTA_RESETVAL                                            (0x00000000U)

/* MOD_VER */

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_SCHEME_MASK                                         (0xC0000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_SCHEME_SHIFT                                        (0x0000001EU)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_SCHEME_RESETVAL                                     (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_SCHEME_MAX                                          (0x00000003U)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_BU_MASK                                             (0x30000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_BU_SHIFT                                            (0x0000001CU)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_BU_RESETVAL                                         (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_BU_MAX                                              (0x00000003U)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MODULE_ID_MASK                                      (0x0FFF0000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MODULE_ID_SHIFT                                     (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MODULE_ID_RESETVAL                                  (0x00000884U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MODULE_ID_MAX                                       (0x00000FFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_RTL_VERSION_MASK                                    (0x0000F800U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_RTL_VERSION_SHIFT                                   (0x0000000BU)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_RTL_VERSION_RESETVAL                                (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_RTL_VERSION_MAX                                     (0x0000001FU)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MAJOR_REVISION_MASK                                 (0x00000700U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MAJOR_REVISION_SHIFT                                (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MAJOR_REVISION_RESETVAL                             (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MAJOR_REVISION_MAX                                  (0x00000007U)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_CUSTOM_REVISION_MASK                                (0x000000C0U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_CUSTOM_REVISION_SHIFT                               (0x00000006U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_CUSTOM_REVISION_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_CUSTOM_REVISION_MAX                                 (0x00000003U)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MINOR_REVISION_MASK                                 (0x0000003FU)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MINOR_REVISION_SHIFT                                (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MINOR_REVISION_RESETVAL                             (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_MINOR_REVISION_MAX                                  (0x0000003FU)

#define CSL_RL2_OF_R5FSS0_CORE1_MOD_VER_RESETVAL                                            (0x68841900U)

/* L2_CTRL */

#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_ENABLE_MASK                                         (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_ENABLE_SHIFT                                        (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_ENABLE_RESETVAL                                     (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_ENABLE_MAX                                          (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_SIZE_MASK                                           (0x00000007U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_SIZE_SHIFT                                          (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_SIZE_RESETVAL                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_SIZE_MAX                                            (0x00000007U)

#define CSL_RL2_OF_R5FSS0_CORE1_L2_CTRL_RESETVAL                                            (0x00000000U)

/* L2_STS */

#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS_OK_TO_GO_MASK                                        (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS_OK_TO_GO_SHIFT                                       (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS_OK_TO_GO_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS_OK_TO_GO_MAX                                         (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_L2_STS_RESETVAL                                             (0x00000000U)

/* L2_LO */

#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO_ADDRESS_LO_LSW_MASK                                   (0xFFFFF800U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO_ADDRESS_LO_LSW_SHIFT                                  (0x0000000BU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO_ADDRESS_LO_LSW_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO_ADDRESS_LO_LSW_MAX                                    (0x001FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_L2_LO_RESETVAL                                              (0x00000000U)

/* L2_HI */

#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI_ADDRESS_HI_LSW_MASK                                   (0xFFFFF800U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI_ADDRESS_HI_LSW_SHIFT                                  (0x0000000BU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI_ADDRESS_HI_LSW_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI_ADDRESS_HI_LSW_MAX                                    (0x001FFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_L2_HI_RESETVAL                                              (0x00000000U)

/* L2HC */

#define CSL_RL2_OF_R5FSS0_CORE1_L2HC_HIT_MASK                                               (0xFFFFFFFFU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2HC_HIT_SHIFT                                              (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2HC_HIT_RESETVAL                                           (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2HC_HIT_MAX                                                (0xFFFFFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_L2HC_RESETVAL                                               (0x00000000U)

/* L2MC */

#define CSL_RL2_OF_R5FSS0_CORE1_L2MC_MISS_MASK                                              (0xFFFFFFFFU)
#define CSL_RL2_OF_R5FSS0_CORE1_L2MC_MISS_SHIFT                                             (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2MC_MISS_RESETVAL                                          (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_L2MC_MISS_MAX                                               (0xFFFFFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_L2MC_RESETVAL                                               (0x00000000U)

/* IRQSTATUS_RAW */

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_DON_MASK                                  (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_DON_SHIFT                                 (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_DON_RESETVAL                              (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_DON_MAX                                   (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_WRERR_MASK                                (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_WRERR_SHIFT                               (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_WRERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_WRERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_RDERR_MASK                                (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_RDERR_SHIFT                               (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_RDERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_FLC_RDERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_HIT_MASK                                   (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_HIT_SHIFT                                  (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_HIT_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_HIT_MAX                                    (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_ERR_MASK                                   (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_ERR_SHIFT                                  (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_ERR_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_WR_ERR_MAX                                    (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_RAW_RESETVAL                                      (0x00000000U)

/* IRQSTATUS_MSK */

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_DON_MASK                                  (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_DON_SHIFT                                 (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_DON_RESETVAL                              (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_DON_MAX                                   (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_WRERR_MASK                                (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_WRERR_SHIFT                               (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_WRERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_WRERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_RDERR_MASK                                (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_RDERR_SHIFT                               (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_RDERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_FLC_RDERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_HIT_MASK                                   (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_HIT_SHIFT                                  (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_HIT_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_HIT_MAX                                    (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_ERR_MASK                                   (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_ERR_SHIFT                                  (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_ERR_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_WR_ERR_MAX                                    (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQSTATUS_MSK_RESETVAL                                      (0x00000000U)

/* IRQENABLE_SET */

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_DON_MASK                               (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_DON_SHIFT                              (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_DON_RESETVAL                           (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_DON_MAX                                (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_WRERR_MASK                             (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_WRERR_SHIFT                            (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_WRERR_RESETVAL                         (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_WRERR_MAX                              (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_RDERR_MASK                             (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_RDERR_SHIFT                            (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_RDERR_RESETVAL                         (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_FLC_RDERR_MAX                              (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_HIT_MASK                                (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_HIT_SHIFT                               (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_HIT_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_HIT_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_ERR_MASK                                (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_ERR_SHIFT                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_ERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_EN_WR_ERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_SET_RESETVAL                                      (0x00000000U)

/* IRQENABLE_CLR */

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_DON_MASK                               (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_DON_SHIFT                              (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_DON_RESETVAL                           (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_DON_MAX                                (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_WRERR_MASK                             (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_WRERR_SHIFT                            (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_WRERR_RESETVAL                         (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_WRERR_MAX                              (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_RDERR_MASK                             (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_RDERR_SHIFT                            (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_RDERR_RESETVAL                         (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_FLC_RDERR_MAX                              (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_HIT_MASK                                (0x00000002U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_HIT_SHIFT                               (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_HIT_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_HIT_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_ERR_MASK                                (0x00000001U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_ERR_SHIFT                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_ERR_RESETVAL                            (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_EN_WR_ERR_MAX                                 (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_IRQENABLE_CLR_RESETVAL                                      (0x00000000U)

/* FLC_CFG */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FIFO_BYPASS_MASK                                    (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FIFO_BYPASS_SHIFT                                   (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FIFO_BYPASS_RESETVAL                                (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FIFO_BYPASS_MAX                                     (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FLC_EXCNT_MASK                                      (0x07000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FLC_EXCNT_SHIFT                                     (0x00000018U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FLC_EXCNT_RESETVAL                                  (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_FLC_EXCNT_MAX                                       (0x00000007U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_ASMNUM_MASK                                         (0x0000FF00U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_ASMNUM_SHIFT                                        (0x00000008U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_ASMNUM_RESETVAL                                     (0x00000005U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_ASMNUM_MAX                                          (0x000000FFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_RANGES_MASK                                         (0x000000FFU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_RANGES_SHIFT                                        (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_RANGES_RESETVAL                                     (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_RANGES_MAX                                          (0x000000FFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_CFG_RESETVAL                                            (0x00000504U)

/* FLC_STS */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS_CPYCMP_MASK                                         (0x0000000FU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS_CPYCMP_SHIFT                                        (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS_CPYCMP_RESETVAL                                     (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS_CPYCMP_MAX                                          (0x0000000FU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_STS_RESETVAL                                            (0x00000000U)

/* FLC_DBG0 */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLCIF_MASK                                         (0x80000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLCIF_SHIFT                                        (0x0000001FU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLCIF_RESETVAL                                     (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLCIF_MAX                                          (0x00000001U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLC_OUT_CNT_MASK                                   (0x07000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLC_OUT_CNT_SHIFT                                  (0x00000018U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLC_OUT_CNT_RESETVAL                               (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_FLC_OUT_CNT_MAX                                    (0x00000007U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_CURFLC_MASK                                        (0x00000003U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_CURFLC_SHIFT                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_CURFLC_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_CURFLC_MAX                                         (0x00000003U)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG0_RESETVAL                                           (0x00000000U)

/* FLC_DBG1 */

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1_FLCADR_MASK                                        (0xFFFFFFFFU)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1_FLCADR_SHIFT                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1_FLCADR_RESETVAL                                    (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1_FLCADR_MAX                                         (0xFFFFFFFFU)

#define CSL_RL2_OF_R5FSS0_CORE1_FLC_DBG1_RESETVAL                                           (0x00000000U)

/* RAT_CFG */

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_ADDR_WIDTH_MASK                                     (0x00FF0000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_ADDR_WIDTH_SHIFT                                    (0x00000010U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_ADDR_WIDTH_RESETVAL                                 (0x00000020U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_ADDR_WIDTH_MAX                                      (0x000000FFU)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_REGIONS_MASK                                        (0x000000FFU)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_REGIONS_SHIFT                                       (0x00000000U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_REGIONS_RESETVAL                                    (0x00000004U)
#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_REGIONS_MAX                                         (0x000000FFU)

#define CSL_RL2_OF_R5FSS0_CORE1_RAT_CFG_RESETVAL                                            (0x00200004U)

#ifdef __cplusplus
}
#endif
#endif
