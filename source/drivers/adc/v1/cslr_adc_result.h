/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef CSLR_ADC_RESULT_H_
#define CSLR_ADC_RESULT_H_

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
    volatile uint16_t ADCRESULT0;
    volatile uint16_t ADCRESULT1;
    volatile uint16_t ADCRESULT2;
    volatile uint16_t ADCRESULT3;
    volatile uint16_t ADCRESULT4;
    volatile uint16_t ADCRESULT5;
    volatile uint16_t ADCRESULT6;
    volatile uint16_t ADCRESULT7;
    volatile uint16_t ADCRESULT8;
    volatile uint16_t ADCRESULT9;
    volatile uint16_t ADCRESULT10;
    volatile uint16_t ADCRESULT11;
    volatile uint16_t ADCRESULT12;
    volatile uint16_t ADCRESULT13;
    volatile uint16_t ADCRESULT14;
    volatile uint16_t ADCRESULT15;
    volatile uint32_t ADCPPB1RESULT;
    volatile uint32_t ADCPPB2RESULT;
    volatile uint32_t ADCPPB3RESULT;
    volatile uint32_t ADCPPB4RESULT;
} CSL_adc_resultRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ADC_RESULT_ADCRESULT0                                              (0x00000000U)
#define CSL_ADC_RESULT_ADCRESULT1                                              (0x00000002U)
#define CSL_ADC_RESULT_ADCRESULT2                                              (0x00000004U)
#define CSL_ADC_RESULT_ADCRESULT3                                              (0x00000006U)
#define CSL_ADC_RESULT_ADCRESULT4                                              (0x00000008U)
#define CSL_ADC_RESULT_ADCRESULT5                                              (0x0000000AU)
#define CSL_ADC_RESULT_ADCRESULT6                                              (0x0000000CU)
#define CSL_ADC_RESULT_ADCRESULT7                                              (0x0000000EU)
#define CSL_ADC_RESULT_ADCRESULT8                                              (0x00000010U)
#define CSL_ADC_RESULT_ADCRESULT9                                              (0x00000012U)
#define CSL_ADC_RESULT_ADCRESULT10                                             (0x00000014U)
#define CSL_ADC_RESULT_ADCRESULT11                                             (0x00000016U)
#define CSL_ADC_RESULT_ADCRESULT12                                             (0x00000018U)
#define CSL_ADC_RESULT_ADCRESULT13                                             (0x0000001AU)
#define CSL_ADC_RESULT_ADCRESULT14                                             (0x0000001CU)
#define CSL_ADC_RESULT_ADCRESULT15                                             (0x0000001EU)
#define CSL_ADC_RESULT_ADCPPB1RESULT                                           (0x00000020U)
#define CSL_ADC_RESULT_ADCPPB2RESULT                                           (0x00000024U)
#define CSL_ADC_RESULT_ADCPPB3RESULT                                           (0x00000028U)
#define CSL_ADC_RESULT_ADCPPB4RESULT                                           (0x0000002CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ADCRESULT0 */

#define CSL_ADC_RESULT_ADCRESULT0_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT0_RESETVAL                                     (0x0000U)

/* ADCRESULT1 */

#define CSL_ADC_RESULT_ADCRESULT1_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT1_RESETVAL                                     (0x0000U)

/* ADCRESULT2 */

#define CSL_ADC_RESULT_ADCRESULT2_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT2_RESETVAL                                     (0x0000U)

/* ADCRESULT3 */

#define CSL_ADC_RESULT_ADCRESULT3_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT3_RESETVAL                                     (0x0000U)

/* ADCRESULT4 */

#define CSL_ADC_RESULT_ADCRESULT4_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT4_RESETVAL                                     (0x0000U)

/* ADCRESULT5 */

#define CSL_ADC_RESULT_ADCRESULT5_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT5_RESETVAL                                     (0x0000U)

/* ADCRESULT6 */

#define CSL_ADC_RESULT_ADCRESULT6_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT6_RESETVAL                                     (0x0000U)

/* ADCRESULT7 */

#define CSL_ADC_RESULT_ADCRESULT7_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT7_RESETVAL                                     (0x0000U)

/* ADCRESULT8 */

#define CSL_ADC_RESULT_ADCRESULT8_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT8_RESETVAL                                     (0x0000U)

/* ADCRESULT9 */

#define CSL_ADC_RESULT_ADCRESULT9_RESULT_MASK                                  (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_SHIFT                                 (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_RESETVAL                              (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_MAX                                   (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT9_RESETVAL                                     (0x0000U)

/* ADCRESULT10 */

#define CSL_ADC_RESULT_ADCRESULT10_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT10_RESETVAL                                    (0x0000U)

/* ADCRESULT11 */

#define CSL_ADC_RESULT_ADCRESULT11_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT11_RESETVAL                                    (0x0000U)

/* ADCRESULT12 */

#define CSL_ADC_RESULT_ADCRESULT12_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT12_RESETVAL                                    (0x0000U)

/* ADCRESULT13 */

#define CSL_ADC_RESULT_ADCRESULT13_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT13_RESETVAL                                    (0x0000U)

/* ADCRESULT14 */

#define CSL_ADC_RESULT_ADCRESULT14_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT14_RESETVAL                                    (0x0000U)

/* ADCRESULT15 */

#define CSL_ADC_RESULT_ADCRESULT15_RESULT_MASK                                 (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_SHIFT                                (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_RESETVAL                             (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_MAX                                  (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT15_RESETVAL                                    (0x0000U)

/* ADCPPB1RESULT */

#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_MASK                            (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_SHIFT                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_MASK                                 (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_SHIFT                                (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_RESETVAL                             (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_MAX                                  (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB1RESULT_RESETVAL                                  (0x00000000U)

/* ADCPPB2RESULT */

#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_MASK                            (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_SHIFT                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_MASK                                 (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_SHIFT                                (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_RESETVAL                             (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_MAX                                  (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB2RESULT_RESETVAL                                  (0x00000000U)

/* ADCPPB3RESULT */

#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_MASK                            (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_SHIFT                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_MASK                                 (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_SHIFT                                (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_RESETVAL                             (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_MAX                                  (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB3RESULT_RESETVAL                                  (0x00000000U)

/* ADCPPB4RESULT */

#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_MASK                            (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_SHIFT                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_MASK                                 (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_SHIFT                                (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_RESETVAL                             (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_MAX                                  (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB4RESULT_RESETVAL                                  (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
