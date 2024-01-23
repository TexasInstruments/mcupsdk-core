/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  Name        : cslr_adc_safety_aggr.h
*/
#ifndef CSLR_ADC_SAFETY_AGGR_H_
#define CSLR_ADC_SAFETY_AGGR_H_

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
    volatile uint32_t OOTFLG;
    volatile uint32_t OOTFLGCLR;
    volatile uint32_t RES1OVF;
    volatile uint32_t RES1OVFCLR;
    volatile uint32_t RES2OVF;
    volatile uint32_t RES2OVFCLR;
    volatile uint16_t CHECKINTFLG;
    volatile uint8_t  Resv_28[2];
    volatile uint16_t CHECKINTFLGCLR;
    volatile uint8_t  Resv_32[2];
    volatile uint32_t CHECKINTSEL1;
    volatile uint32_t CHECKINTSEL2;
    volatile uint32_t CHECKINTSEL3;
    volatile uint8_t  Resv_48[4];
    volatile uint32_t CHECKEVT1SEL1;
    volatile uint32_t CHECKEVT1SEL2;
    volatile uint32_t CHECKEVT1SEL3;
    volatile uint8_t  Resv_64[4];
    volatile uint32_t CHECKEVT2SEL1;
    volatile uint32_t CHECKEVT2SEL2;
    volatile uint32_t CHECKEVT2SEL3;
    volatile uint8_t  Resv_80[4];
    volatile uint32_t CHECKEVT3SEL1;
    volatile uint32_t CHECKEVT3SEL2;
    volatile uint32_t CHECKEVT3SEL3;
    volatile uint8_t  Resv_96[4];
    volatile uint32_t CHECKEVT4SEL1;
    volatile uint32_t CHECKEVT4SEL2;
    volatile uint32_t CHECKEVT4SEL3;
} CSL_adc_safety_aggrRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ADC_SAFETY_AGGR_OOTFLG                                             (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR                                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF                                            (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR                                         (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF                                            (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR                                         (0x00000014U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG                                        (0x00000018U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR                                     (0x0000001CU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1                                       (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2                                       (0x00000024U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3                                       (0x00000028U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1                                      (0x00000030U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2                                      (0x00000034U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3                                      (0x00000038U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1                                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2                                      (0x00000044U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3                                      (0x00000048U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1                                      (0x00000050U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2                                      (0x00000054U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3                                      (0x00000058U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1                                      (0x00000060U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2                                      (0x00000064U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3                                      (0x00000068U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* OOTFLG */

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT1_MASK                                   (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT1_SHIFT                                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT1_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT1_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT2_MASK                                   (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT2_SHIFT                                  (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT2_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT2_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT3_MASK                                   (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT3_SHIFT                                  (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT3_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT3_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT4_MASK                                   (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT4_SHIFT                                  (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT4_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT4_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT5_MASK                                   (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT5_SHIFT                                  (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT5_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT5_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT6_MASK                                   (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT6_SHIFT                                  (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT6_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT6_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT7_MASK                                   (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT7_SHIFT                                  (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT7_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT7_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT8_MASK                                   (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT8_SHIFT                                  (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT8_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT8_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT9_MASK                                   (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT9_SHIFT                                  (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT9_RESETVAL                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT9_MAX                                    (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT10_MASK                                  (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT10_SHIFT                                 (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT10_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT10_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT11_MASK                                  (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT11_SHIFT                                 (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT11_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT11_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT12_MASK                                  (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT12_SHIFT                                 (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT12_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT12_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT13_MASK                                  (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT13_SHIFT                                 (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT13_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT13_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT14_MASK                                  (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT14_SHIFT                                 (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT14_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT14_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT15_MASK                                  (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT15_SHIFT                                 (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT15_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT15_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT16_MASK                                  (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT16_SHIFT                                 (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT16_RESETVAL                              (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_OOT16_MAX                                   (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_RESERVED_1_MASK                             (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_RESERVED_1_SHIFT                            (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_RESERVED_1_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLG_RESERVED_1_MAX                              (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_OOTFLG_RESETVAL                                    (0x00000000U)

/* OOTFLGCLR */

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT1_MASK                                (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT1_SHIFT                               (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT1_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT1_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT2_MASK                                (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT2_SHIFT                               (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT2_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT2_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT3_MASK                                (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT3_SHIFT                               (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT3_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT3_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT4_MASK                                (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT4_SHIFT                               (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT4_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT4_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT5_MASK                                (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT5_SHIFT                               (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT5_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT5_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT6_MASK                                (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT6_SHIFT                               (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT6_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT6_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT7_MASK                                (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT7_SHIFT                               (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT7_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT7_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT8_MASK                                (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT8_SHIFT                               (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT8_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT8_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT9_MASK                                (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT9_SHIFT                               (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT9_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT9_MAX                                 (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT10_MASK                               (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT10_SHIFT                              (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT10_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT10_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT11_MASK                               (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT11_SHIFT                              (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT11_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT11_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT12_MASK                               (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT12_SHIFT                              (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT12_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT12_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT13_MASK                               (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT13_SHIFT                              (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT13_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT13_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT14_MASK                               (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT14_SHIFT                              (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT14_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT14_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT15_MASK                               (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT15_SHIFT                              (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT15_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT15_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT16_MASK                               (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT16_SHIFT                              (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT16_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_OOT16_MAX                                (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_RESERVED_1_MASK                          (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_RESERVED_1_SHIFT                         (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_RESERVED_1_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_RESERVED_1_MAX                           (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_OOTFLGCLR_RESETVAL                                 (0x00000000U)

/* RES1OVF */

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF1_MASK                              (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF1_SHIFT                             (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF1_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF1_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF2_MASK                              (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF2_SHIFT                             (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF2_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF2_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF3_MASK                              (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF3_SHIFT                             (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF3_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF3_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF4_MASK                              (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF4_SHIFT                             (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF4_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF4_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF5_MASK                              (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF5_SHIFT                             (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF5_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF5_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF6_MASK                              (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF6_SHIFT                             (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF6_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF6_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF7_MASK                              (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF7_SHIFT                             (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF7_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF7_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF8_MASK                              (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF8_SHIFT                             (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF8_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF8_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF9_MASK                              (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF9_SHIFT                             (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF9_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF9_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF10_MASK                             (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF10_SHIFT                            (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF10_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF10_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF11_MASK                             (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF11_SHIFT                            (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF11_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF11_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF12_MASK                             (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF12_SHIFT                            (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF12_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF12_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF13_MASK                             (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF13_SHIFT                            (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF13_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF13_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF14_MASK                             (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF14_SHIFT                            (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF14_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF14_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF15_MASK                             (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF15_SHIFT                            (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF15_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF15_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF16_MASK                             (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF16_SHIFT                            (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF16_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RES1OVF16_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RESERVED_1_MASK                            (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RESERVED_1_SHIFT                           (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVF_RESERVED_1_MAX                             (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_RES1OVF_RESETVAL                                   (0x00000000U)

/* RES1OVFCLR */

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF1_MASK                           (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF1_SHIFT                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF1_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF1_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF2_MASK                           (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF2_SHIFT                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF2_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF2_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF3_MASK                           (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF3_SHIFT                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF3_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF3_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF4_MASK                           (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF4_SHIFT                          (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF4_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF4_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF5_MASK                           (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF5_SHIFT                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF5_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF5_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF6_MASK                           (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF6_SHIFT                          (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF6_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF6_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF7_MASK                           (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF7_SHIFT                          (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF7_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF7_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF8_MASK                           (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF8_SHIFT                          (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF8_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF8_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF9_MASK                           (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF9_SHIFT                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF9_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF9_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF10_MASK                          (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF10_SHIFT                         (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF10_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF10_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF11_MASK                          (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF11_SHIFT                         (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF11_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF11_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF12_MASK                          (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF12_SHIFT                         (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF12_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF12_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF13_MASK                          (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF13_SHIFT                         (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF13_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF13_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF14_MASK                          (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF14_SHIFT                         (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF14_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF14_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF15_MASK                          (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF15_SHIFT                         (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF15_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF15_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF16_MASK                          (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF16_SHIFT                         (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF16_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RES1OVF16_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RESERVED_1_MASK                         (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RESERVED_1_SHIFT                        (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RESERVED_1_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RESERVED_1_MAX                          (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_RES1OVFCLR_RESETVAL                                (0x00000000U)

/* RES2OVF */

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF1_MASK                              (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF1_SHIFT                             (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF1_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF1_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF2_MASK                              (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF2_SHIFT                             (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF2_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF2_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF3_MASK                              (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF3_SHIFT                             (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF3_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF3_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF4_MASK                              (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF4_SHIFT                             (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF4_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF4_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF5_MASK                              (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF5_SHIFT                             (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF5_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF5_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF6_MASK                              (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF6_SHIFT                             (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF6_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF6_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF7_MASK                              (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF7_SHIFT                             (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF7_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF7_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF8_MASK                              (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF8_SHIFT                             (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF8_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF8_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF9_MASK                              (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF9_SHIFT                             (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF9_RESETVAL                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF9_MAX                               (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF10_MASK                             (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF10_SHIFT                            (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF10_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF10_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF11_MASK                             (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF11_SHIFT                            (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF11_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF11_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF12_MASK                             (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF12_SHIFT                            (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF12_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF12_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF13_MASK                             (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF13_SHIFT                            (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF13_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF13_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF14_MASK                             (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF14_SHIFT                            (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF14_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF14_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF15_MASK                             (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF15_SHIFT                            (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF15_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF15_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF16_MASK                             (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF16_SHIFT                            (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF16_RESETVAL                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RES2OVF16_MAX                              (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RESERVED_1_MASK                            (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RESERVED_1_SHIFT                           (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVF_RESERVED_1_MAX                             (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_RES2OVF_RESETVAL                                   (0x00000000U)

/* RES2OVFCLR */

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF1_MASK                           (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF1_SHIFT                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF1_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF1_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF2_MASK                           (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF2_SHIFT                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF2_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF2_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF3_MASK                           (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF3_SHIFT                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF3_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF3_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF4_MASK                           (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF4_SHIFT                          (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF4_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF4_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF5_MASK                           (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF5_SHIFT                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF5_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF5_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF6_MASK                           (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF6_SHIFT                          (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF6_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF6_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF7_MASK                           (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF7_SHIFT                          (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF7_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF7_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF8_MASK                           (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF8_SHIFT                          (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF8_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF8_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF9_MASK                           (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF9_SHIFT                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF9_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF9_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF10_MASK                          (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF10_SHIFT                         (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF10_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF10_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF11_MASK                          (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF11_SHIFT                         (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF11_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF11_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF12_MASK                          (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF12_SHIFT                         (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF12_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF12_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF13_MASK                          (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF13_SHIFT                         (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF13_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF13_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF14_MASK                          (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF14_SHIFT                         (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF14_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF14_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF15_MASK                          (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF15_SHIFT                         (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF15_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF15_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF16_MASK                          (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF16_SHIFT                         (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF16_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RES2OVF16_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RESERVED_1_MASK                         (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RESERVED_1_SHIFT                        (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RESERVED_1_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RESERVED_1_MAX                          (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_RES2OVFCLR_RESETVAL                                (0x00000000U)

/* CHECKINTFLG */

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_CHECKINT_MASK                          (0x0001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_CHECKINT_SHIFT                         (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_CHECKINT_RESETVAL                      (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_CHECKINT_MAX                           (0x0001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_RESERVED_1_MASK                        (0xFFFEU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_RESERVED_1_SHIFT                       (0x0001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_RESERVED_1_MAX                         (0x7FFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLG_RESETVAL                               (0x0000U)

/* CHECKINTFLGCLR */

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_CHECKINTCLR_MASK                    (0x0001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_CHECKINTCLR_SHIFT                   (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_CHECKINTCLR_RESETVAL                (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_CHECKINTCLR_MAX                     (0x0001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_RESERVED_1_MASK                     (0xFFFEU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_RESERVED_1_SHIFT                    (0x0001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_RESERVED_1_RESETVAL                 (0x0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_RESERVED_1_MAX                      (0x7FFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR_RESETVAL                            (0x0000U)

/* CHECKINTSEL1 */

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF1EN_MASK                       (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF1EN_SHIFT                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF1EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF1EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF2EN_MASK                       (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF2EN_SHIFT                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF2EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF2EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF3EN_MASK                       (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF3EN_SHIFT                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF3EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF3EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF4EN_MASK                       (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF4EN_SHIFT                      (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF4EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF4EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF5EN_MASK                       (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF5EN_SHIFT                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF5EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF5EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF6EN_MASK                       (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF6EN_SHIFT                      (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF6EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF6EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF7EN_MASK                       (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF7EN_SHIFT                      (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF7EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF7EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF8EN_MASK                       (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF8EN_SHIFT                      (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF8EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF8EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF9EN_MASK                       (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF9EN_SHIFT                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF9EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF9EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF10EN_MASK                      (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF10EN_SHIFT                     (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF10EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF10EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF11EN_MASK                      (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF11EN_SHIFT                     (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF11EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF11EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF12EN_MASK                      (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF12EN_SHIFT                     (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF12EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF12EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF13EN_MASK                      (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF13EN_SHIFT                     (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF13EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF13EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF14EN_MASK                      (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF14EN_SHIFT                     (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF14EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF14EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF15EN_MASK                      (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF15EN_SHIFT                     (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF15EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF15EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF16EN_MASK                      (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF16EN_SHIFT                     (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF16EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RES1OVF16EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RESERVED_1_MASK                       (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RESERVED_1_SHIFT                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RESERVED_1_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RESERVED_1_MAX                        (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL1_RESETVAL                              (0x00000000U)

/* CHECKINTSEL2 */

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF1EN_MASK                       (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF1EN_SHIFT                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF1EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF1EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF2EN_MASK                       (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF2EN_SHIFT                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF2EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF2EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF3EN_MASK                       (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF3EN_SHIFT                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF3EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF3EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF4EN_MASK                       (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF4EN_SHIFT                      (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF4EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF4EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF5EN_MASK                       (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF5EN_SHIFT                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF5EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF5EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF6EN_MASK                       (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF6EN_SHIFT                      (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF6EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF6EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF7EN_MASK                       (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF7EN_SHIFT                      (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF7EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF7EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF8EN_MASK                       (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF8EN_SHIFT                      (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF8EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF8EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF9EN_MASK                       (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF9EN_SHIFT                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF9EN_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF9EN_MAX                        (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF10EN_MASK                      (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF10EN_SHIFT                     (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF10EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF10EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF11EN_MASK                      (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF11EN_SHIFT                     (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF11EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF11EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF12EN_MASK                      (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF12EN_SHIFT                     (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF12EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF12EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF13EN_MASK                      (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF13EN_SHIFT                     (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF13EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF13EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF14EN_MASK                      (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF14EN_SHIFT                     (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF14EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF14EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF15EN_MASK                      (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF15EN_SHIFT                     (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF15EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF15EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF16EN_MASK                      (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF16EN_SHIFT                     (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF16EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RES2OVF16EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RESERVED_1_MASK                       (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RESERVED_1_SHIFT                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RESERVED_1_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RESERVED_1_MAX                        (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL2_RESETVAL                              (0x00000000U)

/* CHECKINTSEL3 */

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT1EN_MASK                           (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT1EN_SHIFT                          (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT1EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT1EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT2EN_MASK                           (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT2EN_SHIFT                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT2EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT2EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT3EN_MASK                           (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT3EN_SHIFT                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT3EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT3EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT4EN_MASK                           (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT4EN_SHIFT                          (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT4EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT4EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT5EN_MASK                           (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT5EN_SHIFT                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT5EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT5EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT6EN_MASK                           (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT6EN_SHIFT                          (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT6EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT6EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT7EN_MASK                           (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT7EN_SHIFT                          (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT7EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT7EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT8EN_MASK                           (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT8EN_SHIFT                          (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT8EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT8EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT9EN_MASK                           (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT9EN_SHIFT                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT9EN_RESETVAL                       (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT9EN_MAX                            (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT10EN_MASK                          (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT10EN_SHIFT                         (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT10EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT10EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT11EN_MASK                          (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT11EN_SHIFT                         (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT11EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT11EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT12EN_MASK                          (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT12EN_SHIFT                         (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT12EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT12EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT13EN_MASK                          (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT13EN_SHIFT                         (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT13EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT13EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT14EN_MASK                          (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT14EN_SHIFT                         (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT14EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT14EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT15EN_MASK                          (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT15EN_SHIFT                         (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT15EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT15EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT16EN_MASK                          (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT16EN_SHIFT                         (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT16EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_OOT16EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_RESERVED_1_MASK                       (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_RESERVED_1_SHIFT                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_RESERVED_1_RESETVAL                   (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_RESERVED_1_MAX                        (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKINTSEL3_RESETVAL                              (0x00000000U)

/* CHECKEVT1SEL1 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RES1OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1_RESETVAL                             (0x00000000U)

/* CHECKEVT1SEL2 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RES2OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL2_RESETVAL                             (0x00000000U)

/* CHECKEVT1SEL3 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT1EN_MASK                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT1EN_SHIFT                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT1EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT1EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT2EN_MASK                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT2EN_SHIFT                         (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT2EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT2EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT3EN_MASK                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT3EN_SHIFT                         (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT3EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT3EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT4EN_MASK                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT4EN_SHIFT                         (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT4EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT4EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT5EN_MASK                          (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT5EN_SHIFT                         (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT5EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT5EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT6EN_MASK                          (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT6EN_SHIFT                         (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT6EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT6EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT7EN_MASK                          (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT7EN_SHIFT                         (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT7EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT7EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT8EN_MASK                          (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT8EN_SHIFT                         (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT8EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT8EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT9EN_MASK                          (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT9EN_SHIFT                         (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT9EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT9EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT10EN_MASK                         (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT10EN_SHIFT                        (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT10EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT10EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT11EN_MASK                         (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT11EN_SHIFT                        (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT11EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT11EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT12EN_MASK                         (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT12EN_SHIFT                        (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT12EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT12EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT13EN_MASK                         (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT13EN_SHIFT                        (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT13EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT13EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT14EN_MASK                         (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT14EN_SHIFT                        (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT14EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT14EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT15EN_MASK                         (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT15EN_SHIFT                        (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT15EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT15EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT16EN_MASK                         (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT16EN_SHIFT                        (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT16EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_OOT16EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL3_RESETVAL                             (0x00000000U)

/* CHECKEVT2SEL1 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RES1OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL1_RESETVAL                             (0x00000000U)

/* CHECKEVT2SEL2 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RES2OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL2_RESETVAL                             (0x00000000U)

/* CHECKEVT2SEL3 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT1EN_MASK                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT1EN_SHIFT                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT1EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT1EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT2EN_MASK                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT2EN_SHIFT                         (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT2EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT2EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT3EN_MASK                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT3EN_SHIFT                         (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT3EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT3EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT4EN_MASK                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT4EN_SHIFT                         (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT4EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT4EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT5EN_MASK                          (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT5EN_SHIFT                         (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT5EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT5EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT6EN_MASK                          (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT6EN_SHIFT                         (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT6EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT6EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT7EN_MASK                          (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT7EN_SHIFT                         (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT7EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT7EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT8EN_MASK                          (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT8EN_SHIFT                         (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT8EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT8EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT9EN_MASK                          (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT9EN_SHIFT                         (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT9EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT9EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT10EN_MASK                         (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT10EN_SHIFT                        (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT10EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT10EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT11EN_MASK                         (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT11EN_SHIFT                        (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT11EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT11EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT12EN_MASK                         (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT12EN_SHIFT                        (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT12EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT12EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT13EN_MASK                         (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT13EN_SHIFT                        (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT13EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT13EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT14EN_MASK                         (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT14EN_SHIFT                        (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT14EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT14EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT15EN_MASK                         (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT15EN_SHIFT                        (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT15EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT15EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT16EN_MASK                         (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT16EN_SHIFT                        (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT16EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_OOT16EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT2SEL3_RESETVAL                             (0x00000000U)

/* CHECKEVT3SEL1 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RES1OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL1_RESETVAL                             (0x00000000U)

/* CHECKEVT3SEL2 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RES2OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL2_RESETVAL                             (0x00000000U)

/* CHECKEVT3SEL3 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT1EN_MASK                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT1EN_SHIFT                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT1EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT1EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT2EN_MASK                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT2EN_SHIFT                         (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT2EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT2EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT3EN_MASK                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT3EN_SHIFT                         (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT3EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT3EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT4EN_MASK                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT4EN_SHIFT                         (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT4EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT4EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT5EN_MASK                          (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT5EN_SHIFT                         (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT5EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT5EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT6EN_MASK                          (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT6EN_SHIFT                         (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT6EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT6EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT7EN_MASK                          (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT7EN_SHIFT                         (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT7EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT7EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT8EN_MASK                          (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT8EN_SHIFT                         (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT8EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT8EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT9EN_MASK                          (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT9EN_SHIFT                         (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT9EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT9EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT10EN_MASK                         (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT10EN_SHIFT                        (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT10EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT10EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT11EN_MASK                         (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT11EN_SHIFT                        (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT11EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT11EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT12EN_MASK                         (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT12EN_SHIFT                        (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT12EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT12EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT13EN_MASK                         (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT13EN_SHIFT                        (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT13EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT13EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT14EN_MASK                         (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT14EN_SHIFT                        (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT14EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT14EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT15EN_MASK                         (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT15EN_SHIFT                        (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT15EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT15EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT16EN_MASK                         (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT16EN_SHIFT                        (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT16EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_OOT16EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT3SEL3_RESETVAL                             (0x00000000U)

/* CHECKEVT4SEL1 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RES1OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL1_RESETVAL                             (0x00000000U)

/* CHECKEVT4SEL2 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF1EN_MASK                      (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF1EN_SHIFT                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF1EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF1EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF2EN_MASK                      (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF2EN_SHIFT                     (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF2EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF2EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF3EN_MASK                      (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF3EN_SHIFT                     (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF3EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF3EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF4EN_MASK                      (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF4EN_SHIFT                     (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF4EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF4EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF5EN_MASK                      (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF5EN_SHIFT                     (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF5EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF5EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF6EN_MASK                      (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF6EN_SHIFT                     (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF6EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF6EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF7EN_MASK                      (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF7EN_SHIFT                     (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF7EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF7EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF8EN_MASK                      (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF8EN_SHIFT                     (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF8EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF8EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF9EN_MASK                      (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF9EN_SHIFT                     (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF9EN_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF9EN_MAX                       (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF10EN_MASK                     (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF10EN_SHIFT                    (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF10EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF10EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF11EN_MASK                     (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF11EN_SHIFT                    (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF11EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF11EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF12EN_MASK                     (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF12EN_SHIFT                    (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF12EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF12EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF13EN_MASK                     (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF13EN_SHIFT                    (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF13EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF13EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF14EN_MASK                     (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF14EN_SHIFT                    (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF14EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF14EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF15EN_MASK                     (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF15EN_SHIFT                    (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF15EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF15EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF16EN_MASK                     (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF16EN_SHIFT                    (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF16EN_RESETVAL                 (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RES2OVF16EN_MAX                      (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL2_RESETVAL                             (0x00000000U)

/* CHECKEVT4SEL3 */

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT1EN_MASK                          (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT1EN_SHIFT                         (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT1EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT1EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT2EN_MASK                          (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT2EN_SHIFT                         (0x00000001U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT2EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT2EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT3EN_MASK                          (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT3EN_SHIFT                         (0x00000002U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT3EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT3EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT4EN_MASK                          (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT4EN_SHIFT                         (0x00000003U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT4EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT4EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT5EN_MASK                          (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT5EN_SHIFT                         (0x00000004U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT5EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT5EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT6EN_MASK                          (0x00000020U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT6EN_SHIFT                         (0x00000005U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT6EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT6EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT7EN_MASK                          (0x00000040U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT7EN_SHIFT                         (0x00000006U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT7EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT7EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT8EN_MASK                          (0x00000080U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT8EN_SHIFT                         (0x00000007U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT8EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT8EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT9EN_MASK                          (0x00000100U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT9EN_SHIFT                         (0x00000008U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT9EN_RESETVAL                      (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT9EN_MAX                           (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT10EN_MASK                         (0x00000200U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT10EN_SHIFT                        (0x00000009U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT10EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT10EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT11EN_MASK                         (0x00000400U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT11EN_SHIFT                        (0x0000000AU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT11EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT11EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT12EN_MASK                         (0x00000800U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT12EN_SHIFT                        (0x0000000BU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT12EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT12EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT13EN_MASK                         (0x00001000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT13EN_SHIFT                        (0x0000000CU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT13EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT13EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT14EN_MASK                         (0x00002000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT14EN_SHIFT                        (0x0000000DU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT14EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT14EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT15EN_MASK                         (0x00004000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT15EN_SHIFT                        (0x0000000EU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT15EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT15EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT16EN_MASK                         (0x00008000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT16EN_SHIFT                        (0x0000000FU)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT16EN_RESETVAL                     (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_OOT16EN_MAX                          (0x00000001U)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_RESERVED_1_MASK                      (0xFFFF0000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_RESERVED_1_SHIFT                     (0x00000010U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_RESERVED_1_RESETVAL                  (0x00000000U)
#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_RESERVED_1_MAX                       (0x0000FFFFU)

#define CSL_ADC_SAFETY_AGGR_CHECKEVT4SEL3_RESETVAL                             (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
