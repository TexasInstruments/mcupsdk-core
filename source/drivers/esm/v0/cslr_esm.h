/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_esm.h
*/
#ifndef CSLR_ESM_H_
#define CSLR_ESM_H_

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
    volatile uint32_t ESMIEPSR1;
    volatile uint32_t ESMIEPCR1;
    volatile uint32_t ESMIESR1;
    volatile uint32_t ESMIECR1;
    volatile uint32_t ESMILSR1;
    volatile uint32_t ESMILCR1;
    volatile uint32_t ESMSR1;
    volatile uint32_t ESMSR2;
    volatile uint32_t ESMSR3;
    volatile uint32_t ESMEPSR;
    volatile uint32_t ESMIOFFHR;
    volatile uint32_t ESMIOFFLR;
    volatile uint32_t ESMLTCR;
    volatile uint32_t ESMLTCPR;
    volatile uint32_t ESMEKR;
    volatile uint32_t ESMSSR2;
    volatile uint32_t ESMIEPSR4;
    volatile uint32_t ESMIEPCR4;
    volatile uint32_t ESMIESR4;
    volatile uint32_t ESMIECR4;
    volatile uint32_t ESMILSR4;
    volatile uint32_t ESMILCR4;
    volatile uint32_t ESMSR4;
    volatile uint8_t  Resv_128[36];
    volatile uint32_t ESMIEPSR7;
    volatile uint32_t ESMIEPCR7;
    volatile uint32_t ESMIESR7;
    volatile uint32_t ESMIECR7;
    volatile uint32_t ESMILSR7;
    volatile uint32_t ESMILCR7;
    volatile uint32_t ESMSR7;
    volatile uint8_t  Resv_192[36];
    volatile uint32_t ESMIEPSR10;
    volatile uint32_t ESMIEPCR10;
    volatile uint32_t ESMIESR10;
    volatile uint32_t ESMIECR10;
    volatile uint32_t ESMILSR10;
    volatile uint32_t ESMILCR10;
    volatile uint32_t ESMSR10;
} CSL_esmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ESM_ESMIEPSR1                                                  (0x00000000U)
#define CSL_ESM_ESMIEPCR1                                                  (0x00000004U)
#define CSL_ESM_ESMIESR1                                                   (0x00000008U)
#define CSL_ESM_ESMIECR1                                                   (0x0000000CU)
#define CSL_ESM_ESMILSR1                                                   (0x00000010U)
#define CSL_ESM_ESMILCR1                                                   (0x00000014U)
#define CSL_ESM_ESMSR1                                                     (0x00000018U)
#define CSL_ESM_ESMSR2                                                     (0x0000001CU)
#define CSL_ESM_ESMSR3                                                     (0x00000020U)
#define CSL_ESM_ESMEPSR                                                    (0x00000024U)
#define CSL_ESM_ESMIOFFHR                                                  (0x00000028U)
#define CSL_ESM_ESMIOFFLR                                                  (0x0000002CU)
#define CSL_ESM_ESMLTCR                                                    (0x00000030U)
#define CSL_ESM_ESMLTCPR                                                   (0x00000034U)
#define CSL_ESM_ESMEKR                                                     (0x00000038U)
#define CSL_ESM_ESMSSR2                                                    (0x0000003CU)
#define CSL_ESM_ESMIEPSR4                                                  (0x00000040U)
#define CSL_ESM_ESMIEPCR4                                                  (0x00000044U)
#define CSL_ESM_ESMIESR4                                                   (0x00000048U)
#define CSL_ESM_ESMIECR4                                                   (0x0000004CU)
#define CSL_ESM_ESMILSR4                                                   (0x00000050U)
#define CSL_ESM_ESMILCR4                                                   (0x00000054U)
#define CSL_ESM_ESMSR4                                                     (0x00000058U)
#define CSL_ESM_ESMIEPSR7                                                  (0x00000080U)
#define CSL_ESM_ESMIEPCR7                                                  (0x00000084U)
#define CSL_ESM_ESMIESR7                                                   (0x00000088U)
#define CSL_ESM_ESMIECR7                                                   (0x0000008CU)
#define CSL_ESM_ESMILSR7                                                   (0x00000090U)
#define CSL_ESM_ESMILCR7                                                   (0x00000094U)
#define CSL_ESM_ESMSR7                                                     (0x00000098U)
#define CSL_ESM_ESMIEPSR10                                                 (0x000000C0U)
#define CSL_ESM_ESMIEPCR10                                                 (0x000000C4U)
#define CSL_ESM_ESMIESR10                                                  (0x000000C8U)
#define CSL_ESM_ESMIECR10                                                  (0x000000CCU)
#define CSL_ESM_ESMILSR10                                                  (0x000000D0U)
#define CSL_ESM_ESMILCR10                                                  (0x000000D4U)
#define CSL_ESM_ESMSR10                                                    (0x000000D8U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ESMIEPSR1 */

#define CSL_ESM_ESMIEPSR1_IEPSET_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPSR1_IEPSET_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPSR1_IEPSET_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPSR1_IEPSET_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPSR1_RESETVAL                                         (0x00000000U)

/* ESMIEPCR1 */

#define CSL_ESM_ESMIEPCR1_IEPCLR_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPCR1_IEPCLR_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPCR1_IEPCLR_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPCR1_IEPCLR_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPCR1_RESETVAL                                         (0x00000000U)

/* ESMIESR1 */

#define CSL_ESM_ESMIESR1_INTENSET_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIESR1_INTENSET_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIESR1_INTENSET_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIESR1_INTENSET_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIESR1_RESETVAL                                          (0x00000000U)

/* ESMIECR1 */

#define CSL_ESM_ESMIECR1_INTENCLR_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIECR1_INTENCLR_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIECR1_INTENCLR_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIECR1_INTENCLR_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIECR1_RESETVAL                                          (0x00000000U)

/* ESMILSR1 */

#define CSL_ESM_ESMILSR1_INTLVLSET_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILSR1_INTLVLSET_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILSR1_INTLVLSET_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILSR1_INTLVLSET_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILSR1_RESETVAL                                          (0x00000000U)

/* ESMILCR1 */

#define CSL_ESM_ESMILCR1_INTLVLCLR_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILCR1_INTLVLCLR_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILCR1_INTLVLCLR_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILCR1_INTLVLCLR_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILCR1_RESETVAL                                          (0x00000000U)

/* ESMSR1 */

#define CSL_ESM_ESMSR1_ESF_MASK                                            (0xFFFFFFFFU)
#define CSL_ESM_ESMSR1_ESF_SHIFT                                           (0x00000000U)
#define CSL_ESM_ESMSR1_ESF_RESETVAL                                        (0x00000000U)
#define CSL_ESM_ESMSR1_ESF_MAX                                             (0xFFFFFFFFU)

#define CSL_ESM_ESMSR1_RESETVAL                                            (0x00000000U)

/* ESMSR2 */

#define CSL_ESM_ESMSR2_ESF_MASK                                            (0xFFFFFFFFU)
#define CSL_ESM_ESMSR2_ESF_SHIFT                                           (0x00000000U)
#define CSL_ESM_ESMSR2_ESF_RESETVAL                                        (0x00000000U)
#define CSL_ESM_ESMSR2_ESF_MAX                                             (0xFFFFFFFFU)

#define CSL_ESM_ESMSR2_RESETVAL                                            (0x00000000U)

/* ESMSR3 */

#define CSL_ESM_ESMSR3_ESF_MASK                                            (0xFFFFFFFFU)
#define CSL_ESM_ESMSR3_ESF_SHIFT                                           (0x00000000U)
#define CSL_ESM_ESMSR3_ESF_RESETVAL                                        (0x00000000U)
#define CSL_ESM_ESMSR3_ESF_MAX                                             (0xFFFFFFFFU)

#define CSL_ESM_ESMSR3_RESETVAL                                            (0x00000000U)

/* ESMEPSR */

#define CSL_ESM_ESMEPSR_EPSF_MASK                                          (0x00000001U)
#define CSL_ESM_ESMEPSR_EPSF_SHIFT                                         (0x00000000U)
#define CSL_ESM_ESMEPSR_EPSF_RESETVAL                                      (0x00000000U)
#define CSL_ESM_ESMEPSR_EPSF_MAX                                           (0x00000001U)

#define CSL_ESM_ESMEPSR_RESERVED_MASK                                      (0xFFFFFFFEU)
#define CSL_ESM_ESMEPSR_RESERVED_SHIFT                                     (0x00000001U)
#define CSL_ESM_ESMEPSR_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMEPSR_RESERVED_MAX                                       (0x7FFFFFFFU)

#define CSL_ESM_ESMEPSR_RESETVAL                                           (0x00000000U)

/* ESMIOFFHR */

#define CSL_ESM_ESMIOFFHR_INTOFFH_MASK                                     (0x000001FFU)
#define CSL_ESM_ESMIOFFHR_INTOFFH_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIOFFHR_INTOFFH_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIOFFHR_INTOFFH_MAX                                      (0x000001FFU)

#define CSL_ESM_ESMIOFFHR_RESERVED_MASK                                    (0xFFFFFE00U)
#define CSL_ESM_ESMIOFFHR_RESERVED_SHIFT                                   (0x00000009U)
#define CSL_ESM_ESMIOFFHR_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMIOFFHR_RESERVED_MAX                                     (0x007FFFFFU)

#define CSL_ESM_ESMIOFFHR_RESETVAL                                         (0x00000000U)

/* ESMIOFFLR */

#define CSL_ESM_ESMIOFFLR_INTOFFL_MASK                                     (0x000000FFU)
#define CSL_ESM_ESMIOFFLR_INTOFFL_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIOFFLR_INTOFFL_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIOFFLR_INTOFFL_MAX                                      (0x000000FFU)

#define CSL_ESM_ESMIOFFLR_RESERVED_MASK                                    (0xFFFFFF00U)
#define CSL_ESM_ESMIOFFLR_RESERVED_SHIFT                                   (0x00000008U)
#define CSL_ESM_ESMIOFFLR_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMIOFFLR_RESERVED_MAX                                     (0x00FFFFFFU)

#define CSL_ESM_ESMIOFFLR_RESETVAL                                         (0x00000000U)

/* ESMLTCR */

#define CSL_ESM_ESMLTCR_LTCP_MASK                                          (0x0000FFFFU)
#define CSL_ESM_ESMLTCR_LTCP_SHIFT                                         (0x00000000U)
#define CSL_ESM_ESMLTCR_LTCP_RESETVAL                                      (0x00000000U)
#define CSL_ESM_ESMLTCR_LTCP_MAX                                           (0x0000FFFFU)

#define CSL_ESM_ESMLTCR_RESERVED_MASK                                      (0xFFFF0000U)
#define CSL_ESM_ESMLTCR_RESERVED_SHIFT                                     (0x00000010U)
#define CSL_ESM_ESMLTCR_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMLTCR_RESERVED_MAX                                       (0x0000FFFFU)

#define CSL_ESM_ESMLTCR_RESETVAL                                           (0x00000000U)

/* ESMLTCPR */

#define CSL_ESM_ESMLTCPR_LTCP_MASK                                         (0x0000FFFFU)
#define CSL_ESM_ESMLTCPR_LTCP_SHIFT                                        (0x00000000U)
#define CSL_ESM_ESMLTCPR_LTCP_RESETVAL                                     (0x00000000U)
#define CSL_ESM_ESMLTCPR_LTCP_MAX                                          (0x0000FFFFU)

#define CSL_ESM_ESMLTCPR_RESERVED_MASK                                     (0xFFFF0000U)
#define CSL_ESM_ESMLTCPR_RESERVED_SHIFT                                    (0x00000010U)
#define CSL_ESM_ESMLTCPR_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMLTCPR_RESERVED_MAX                                      (0x0000FFFFU)

#define CSL_ESM_ESMLTCPR_RESETVAL                                          (0x00000000U)

/* ESMEKR */

#define CSL_ESM_ESMEKR_EKEY_MASK                                           (0x0000000FU)
#define CSL_ESM_ESMEKR_EKEY_SHIFT                                          (0x00000000U)
#define CSL_ESM_ESMEKR_EKEY_RESETVAL                                       (0x00000000U)
#define CSL_ESM_ESMEKR_EKEY_MAX                                            (0x0000000FU)

#define CSL_ESM_ESMEKR_RESERVED_MASK                                       (0xFFFFFFF0U)
#define CSL_ESM_ESMEKR_RESERVED_SHIFT                                      (0x00000004U)
#define CSL_ESM_ESMEKR_RESERVED_RESETVAL                                   (0x00000000U)
#define CSL_ESM_ESMEKR_RESERVED_MAX                                        (0x0FFFFFFFU)

#define CSL_ESM_ESMEKR_RESETVAL                                            (0x00000000U)

/* ESMSSR2 */

#define CSL_ESM_ESMSSR2_ESF_MASK                                           (0xFFFFFFFFU)
#define CSL_ESM_ESMSSR2_ESF_SHIFT                                          (0x00000000U)
#define CSL_ESM_ESMSSR2_ESF_RESETVAL                                       (0x00000000U)
#define CSL_ESM_ESMSSR2_ESF_MAX                                            (0xFFFFFFFFU)

#define CSL_ESM_ESMSSR2_RESETVAL                                           (0x00000000U)

/* ESMIEPSR4 */

#define CSL_ESM_ESMIEPSR4_IEPSET_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPSR4_IEPSET_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPSR4_IEPSET_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPSR4_IEPSET_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPSR4_RESETVAL                                         (0x00000000U)

/* ESMIEPCR4 */

#define CSL_ESM_ESMIEPCR4_IEPCLR_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPCR4_IEPCLR_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPCR4_IEPCLR_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPCR4_IEPCLR_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPCR4_RESETVAL                                         (0x00000000U)

/* ESMIESR4 */

#define CSL_ESM_ESMIESR4_INTENSET_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIESR4_INTENSET_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIESR4_INTENSET_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIESR4_INTENSET_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIESR4_RESETVAL                                          (0x00000000U)

/* ESMIECR4 */

#define CSL_ESM_ESMIECR4_INTENCLR_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIECR4_INTENCLR_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIECR4_INTENCLR_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIECR4_INTENCLR_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIECR4_RESETVAL                                          (0x00000000U)

/* ESMILSR4 */

#define CSL_ESM_ESMILSR4_INTLVLSET_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILSR4_INTLVLSET_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILSR4_INTLVLSET_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILSR4_INTLVLSET_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILSR4_RESETVAL                                          (0x00000000U)

/* ESMILCR4 */

#define CSL_ESM_ESMILCR4_INTLVLCLR_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILCR4_INTLVLCLR_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILCR4_INTLVLCLR_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILCR4_INTLVLCLR_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILCR4_RESETVAL                                          (0x00000000U)

/* ESMSR4 */

#define CSL_ESM_ESMSR4_ESF_MASK                                            (0xFFFFFFFFU)
#define CSL_ESM_ESMSR4_ESF_SHIFT                                           (0x00000000U)
#define CSL_ESM_ESMSR4_ESF_RESETVAL                                        (0x00000000U)
#define CSL_ESM_ESMSR4_ESF_MAX                                             (0xFFFFFFFFU)

#define CSL_ESM_ESMSR4_RESETVAL                                            (0x00000000U)

/* ESMIEPSR7 */

#define CSL_ESM_ESMIEPSR7_IEPSET_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPSR7_IEPSET_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPSR7_IEPSET_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPSR7_IEPSET_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPSR7_RESETVAL                                         (0x00000000U)

/* ESMIEPCR7 */

#define CSL_ESM_ESMIEPCR7_IEPCLR_MASK                                      (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPCR7_IEPCLR_SHIFT                                     (0x00000000U)
#define CSL_ESM_ESMIEPCR7_IEPCLR_RESETVAL                                  (0x00000000U)
#define CSL_ESM_ESMIEPCR7_IEPCLR_MAX                                       (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPCR7_RESETVAL                                         (0x00000000U)

/* ESMIESR7 */

#define CSL_ESM_ESMIESR7_INTENSET_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIESR7_INTENSET_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIESR7_INTENSET_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIESR7_INTENSET_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIESR7_RESETVAL                                          (0x00000000U)

/* ESMIECR7 */

#define CSL_ESM_ESMIECR7_INTENCLR_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIECR7_INTENCLR_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIECR7_INTENCLR_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIECR7_INTENCLR_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIECR7_RESETVAL                                          (0x00000000U)

/* ESMILSR7 */

#define CSL_ESM_ESMILSR7_INTLVLSET_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILSR7_INTLVLSET_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILSR7_INTLVLSET_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILSR7_INTLVLSET_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILSR7_RESETVAL                                          (0x00000000U)

/* ESMILCR7 */

#define CSL_ESM_ESMILCR7_INTLVLCLR_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMILCR7_INTLVLCLR_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMILCR7_INTLVLCLR_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMILCR7_INTLVLCLR_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMILCR7_RESETVAL                                          (0x00000000U)

/* ESMSR7 */

#define CSL_ESM_ESMSR7_ESF_MASK                                            (0xFFFFFFFFU)
#define CSL_ESM_ESMSR7_ESF_SHIFT                                           (0x00000000U)
#define CSL_ESM_ESMSR7_ESF_RESETVAL                                        (0x00000000U)
#define CSL_ESM_ESMSR7_ESF_MAX                                             (0xFFFFFFFFU)

#define CSL_ESM_ESMSR7_RESETVAL                                            (0x00000000U)

/* ESMIEPSR10 */

#define CSL_ESM_ESMIEPSR10_IEPSET_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPSR10_IEPSET_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIEPSR10_IEPSET_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIEPSR10_IEPSET_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPSR10_RESETVAL                                        (0x00000000U)

/* ESMIEPCR10 */

#define CSL_ESM_ESMIEPCR10_IEPCLR_MASK                                     (0xFFFFFFFFU)
#define CSL_ESM_ESMIEPCR10_IEPCLR_SHIFT                                    (0x00000000U)
#define CSL_ESM_ESMIEPCR10_IEPCLR_RESETVAL                                 (0x00000000U)
#define CSL_ESM_ESMIEPCR10_IEPCLR_MAX                                      (0xFFFFFFFFU)

#define CSL_ESM_ESMIEPCR10_RESETVAL                                        (0x00000000U)

/* ESMIESR10 */

#define CSL_ESM_ESMIESR10_INTENSET_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMIESR10_INTENSET_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMIESR10_INTENSET_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMIESR10_INTENSET_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMIESR10_RESETVAL                                         (0x00000000U)

/* ESMIECR10 */

#define CSL_ESM_ESMIECR10_INTENCLR_MASK                                    (0xFFFFFFFFU)
#define CSL_ESM_ESMIECR10_INTENCLR_SHIFT                                   (0x00000000U)
#define CSL_ESM_ESMIECR10_INTENCLR_RESETVAL                                (0x00000000U)
#define CSL_ESM_ESMIECR10_INTENCLR_MAX                                     (0xFFFFFFFFU)

#define CSL_ESM_ESMIECR10_RESETVAL                                         (0x00000000U)

/* ESMILSR10 */

#define CSL_ESM_ESMILSR10_INTLVLSET_MASK                                   (0xFFFFFFFFU)
#define CSL_ESM_ESMILSR10_INTLVLSET_SHIFT                                  (0x00000000U)
#define CSL_ESM_ESMILSR10_INTLVLSET_RESETVAL                               (0x00000000U)
#define CSL_ESM_ESMILSR10_INTLVLSET_MAX                                    (0xFFFFFFFFU)

#define CSL_ESM_ESMILSR10_RESETVAL                                         (0x00000000U)

/* ESMILCR10 */

#define CSL_ESM_ESMILCR10_INTLVLCLR_MASK                                   (0xFFFFFFFFU)
#define CSL_ESM_ESMILCR10_INTLVLCLR_SHIFT                                  (0x00000000U)
#define CSL_ESM_ESMILCR10_INTLVLCLR_RESETVAL                               (0x00000000U)
#define CSL_ESM_ESMILCR10_INTLVLCLR_MAX                                    (0xFFFFFFFFU)

#define CSL_ESM_ESMILCR10_RESETVAL                                         (0x00000000U)

/* ESMSR10 */

#define CSL_ESM_ESMSR10_ESF_MASK                                           (0xFFFFFFFFU)
#define CSL_ESM_ESMSR10_ESF_SHIFT                                          (0x00000000U)
#define CSL_ESM_ESMSR10_ESF_RESETVAL                                       (0x00000000U)
#define CSL_ESM_ESMSR10_ESF_MAX                                            (0xFFFFFFFFU)

#define CSL_ESM_ESMSR10_RESETVAL                                           (0x00000000U)


/**************************************************************************
* hw_esm.h alias definitions
**************************************************************************/
#define ESM_ESMIEPSR1                       CSL_ESM_ESMIEPSR1
#define ESM_ESMIEPCR1                       CSL_ESM_ESMIEPCR1
#define ESM_ESMIESR1                        CSL_ESM_ESMIESR1 
#define ESM_ESMIECR1                        CSL_ESM_ESMIECR1 
#define ESM_ESMILSR1                        CSL_ESM_ESMILSR1 
#define ESM_ESMILCR1                        CSL_ESM_ESMILCR1 
#define ESM_ESMSR1                          CSL_ESM_ESMSR1   
#define ESM_ESMSR2                          CSL_ESM_ESMSR2   
#define ESM_ESMSR3                          CSL_ESM_ESMSR3   
#define ESM_ESMEPSR                         CSL_ESM_ESMEPSR  
#define ESM_ESMIOFFHR                       CSL_ESM_ESMIOFFHR
#define ESM_ESMIOFFLR                       CSL_ESM_ESMIOFFLR
#define ESM_ESMLTCR                         CSL_ESM_ESMLTCR  
#define ESM_ESMLTCPR                        CSL_ESM_ESMLTCPR 
#define ESM_ESMEKR                          CSL_ESM_ESMEKR   
#define ESM_ESMSSR2                         CSL_ESM_ESMSSR2  

#define ESM_ESMEPSR_EPSF_MASK               CSL_ESM_ESMEPSR_EPSF_MASK
#define ESM_ESMEPSR_EPSF_SHIFT              CSL_ESM_ESMEPSR_EPSF_SHIFT
#define ESM_ESMIOFFHR_INTOFFH_MASK          CSL_ESM_ESMIOFFHR_INTOFFH_MASK
#define ESM_ESMIOFFHR_INTOFFH_SHIFT         CSL_ESM_ESMIOFFHR_INTOFFH_SHIFT
#define ESM_ESMIOFFLR_INTOFFL_MASK          CSL_ESM_ESMIOFFLR_INTOFFL_MASK
#define ESM_ESMIOFFLR_INTOFFL_SHIFT         CSL_ESM_ESMIOFFLR_INTOFFL_SHIFT
#define ESM_ESMLTCR_LTC_MASK                CSL_ESM_ESMLTCR_LTCP_MASK
#define ESM_ESMLTCR_LTC_SHIFT               CSL_ESM_ESMLTCR_LTCP_SHIFT
#define ESM_ESMLTCPR_LTCPR_MASK             CSL_ESM_ESMLTCPR_LTCP_MASK
#define ESM_ESMLTCPR_LTCPR_SHIFT            CSL_ESM_ESMLTCPR_LTCP_SHIFT
#define ESM_ESMEKR_EKEY_MASK                CSL_ESM_ESMEKR_EKEY_MASK
#define ESM_ESMEKR_EKEY_SHIFT               CSL_ESM_ESMEKR_EKEY_SHIFT

#define ESM_ESMLTCPR_LTCPR_MAX              CSL_ESM_ESMLTCPR_LTCP_MAX

#define ESM_ESMEKR_EKEY_NORMAL_MODE         (0x0U)
#define ESM_ESMEKR_EKEY_ERROR_PIN_RESET     (0x5U)
#define ESM_ESMEKR_EKEY_ERROR_FORCE_MODE    (0xAU)

#ifdef __cplusplus
}
#endif
#endif
