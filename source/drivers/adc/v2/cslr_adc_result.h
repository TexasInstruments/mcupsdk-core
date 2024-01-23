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
 *  Name        : cslr_adc_result.h
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
    volatile uint32_t ADCPPB1SUM;
    volatile uint16_t ADCPPB1COUNT;
    volatile uint8_t  Resv_56[2];
    volatile uint32_t ADCPPB2SUM;
    volatile uint16_t ADCPPB2COUNT;
    volatile uint8_t  Resv_64[2];
    volatile uint32_t ADCPPB3SUM;
    volatile uint16_t ADCPPB3COUNT;
    volatile uint8_t  Resv_72[2];
    volatile uint32_t ADCPPB4SUM;
    volatile uint16_t ADCPPB4COUNT;
    volatile uint8_t  Resv_80[2];
    volatile uint32_t ADCPPB1MAX;
    volatile uint16_t ADCPPB1MAXI;
    volatile uint8_t  Resv_88[2];
    volatile uint32_t ADCPPB1MIN;
    volatile uint16_t ADCPPB1MINI;
    volatile uint8_t  Resv_96[2];
    volatile uint32_t ADCPPB2MAX;
    volatile uint16_t ADCPPB2MAXI;
    volatile uint8_t  Resv_104[2];
    volatile uint32_t ADCPPB2MIN;
    volatile uint16_t ADCPPB2MINI;
    volatile uint8_t  Resv_112[2];
    volatile uint32_t ADCPPB3MAX;
    volatile uint16_t ADCPPB3MAXI;
    volatile uint8_t  Resv_120[2];
    volatile uint32_t ADCPPB3MIN;
    volatile uint16_t ADCPPB3MINI;
    volatile uint8_t  Resv_128[2];
    volatile uint32_t ADCPPB4MAX;
    volatile uint16_t ADCPPB4MAXI;
    volatile uint8_t  Resv_136[2];
    volatile uint32_t ADCPPB4MIN;
    volatile uint16_t ADCPPB4MINI;
} CSL_adc_resultRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ADC_RESULT_ADCRESULT0                                         (0x00000000U)
#define CSL_ADC_RESULT_ADCRESULT1                                         (0x00000002U)
#define CSL_ADC_RESULT_ADCRESULT2                                         (0x00000004U)
#define CSL_ADC_RESULT_ADCRESULT3                                         (0x00000006U)
#define CSL_ADC_RESULT_ADCRESULT4                                         (0x00000008U)
#define CSL_ADC_RESULT_ADCRESULT5                                         (0x0000000AU)
#define CSL_ADC_RESULT_ADCRESULT6                                         (0x0000000CU)
#define CSL_ADC_RESULT_ADCRESULT7                                         (0x0000000EU)
#define CSL_ADC_RESULT_ADCRESULT8                                         (0x00000010U)
#define CSL_ADC_RESULT_ADCRESULT9                                         (0x00000012U)
#define CSL_ADC_RESULT_ADCRESULT10                                        (0x00000014U)
#define CSL_ADC_RESULT_ADCRESULT11                                        (0x00000016U)
#define CSL_ADC_RESULT_ADCRESULT12                                        (0x00000018U)
#define CSL_ADC_RESULT_ADCRESULT13                                        (0x0000001AU)
#define CSL_ADC_RESULT_ADCRESULT14                                        (0x0000001CU)
#define CSL_ADC_RESULT_ADCRESULT15                                        (0x0000001EU)
#define CSL_ADC_RESULT_ADCPPB1RESULT                                      (0x00000020U)
#define CSL_ADC_RESULT_ADCPPB2RESULT                                      (0x00000024U)
#define CSL_ADC_RESULT_ADCPPB3RESULT                                      (0x00000028U)
#define CSL_ADC_RESULT_ADCPPB4RESULT                                      (0x0000002CU)
#define CSL_ADC_RESULT_ADCPPB1SUM                                         (0x00000030U)
#define CSL_ADC_RESULT_ADCPPB1COUNT                                       (0x00000034U)
#define CSL_ADC_RESULT_ADCPPB2SUM                                         (0x00000038U)
#define CSL_ADC_RESULT_ADCPPB2COUNT                                       (0x0000003CU)
#define CSL_ADC_RESULT_ADCPPB3SUM                                         (0x00000040U)
#define CSL_ADC_RESULT_ADCPPB3COUNT                                       (0x00000044U)
#define CSL_ADC_RESULT_ADCPPB4SUM                                         (0x00000048U)
#define CSL_ADC_RESULT_ADCPPB4COUNT                                       (0x0000004CU)
#define CSL_ADC_RESULT_ADCPPB1MAX                                         (0x00000050U)
#define CSL_ADC_RESULT_ADCPPB1MAXI                                        (0x00000054U)
#define CSL_ADC_RESULT_ADCPPB1MIN                                         (0x00000058U)
#define CSL_ADC_RESULT_ADCPPB1MINI                                        (0x0000005CU)
#define CSL_ADC_RESULT_ADCPPB2MAX                                         (0x00000060U)
#define CSL_ADC_RESULT_ADCPPB2MAXI                                        (0x00000064U)
#define CSL_ADC_RESULT_ADCPPB2MIN                                         (0x00000068U)
#define CSL_ADC_RESULT_ADCPPB2MINI                                        (0x0000006CU)
#define CSL_ADC_RESULT_ADCPPB3MAX                                         (0x00000070U)
#define CSL_ADC_RESULT_ADCPPB3MAXI                                        (0x00000074U)
#define CSL_ADC_RESULT_ADCPPB3MIN                                         (0x00000078U)
#define CSL_ADC_RESULT_ADCPPB3MINI                                        (0x0000007CU)
#define CSL_ADC_RESULT_ADCPPB4MAX                                         (0x00000080U)
#define CSL_ADC_RESULT_ADCPPB4MAXI                                        (0x00000084U)
#define CSL_ADC_RESULT_ADCPPB4MIN                                         (0x00000088U)
#define CSL_ADC_RESULT_ADCPPB4MINI                                        (0x0000008CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ADCRESULT0 */

#define CSL_ADC_RESULT_ADCRESULT0_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT0_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT0_RESETVAL                                (0x0000U)

/* ADCRESULT1 */

#define CSL_ADC_RESULT_ADCRESULT1_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT1_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT1_RESETVAL                                (0x0000U)

/* ADCRESULT2 */

#define CSL_ADC_RESULT_ADCRESULT2_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT2_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT2_RESETVAL                                (0x0000U)

/* ADCRESULT3 */

#define CSL_ADC_RESULT_ADCRESULT3_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT3_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT3_RESETVAL                                (0x0000U)

/* ADCRESULT4 */

#define CSL_ADC_RESULT_ADCRESULT4_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT4_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT4_RESETVAL                                (0x0000U)

/* ADCRESULT5 */

#define CSL_ADC_RESULT_ADCRESULT5_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT5_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT5_RESETVAL                                (0x0000U)

/* ADCRESULT6 */

#define CSL_ADC_RESULT_ADCRESULT6_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT6_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT6_RESETVAL                                (0x0000U)

/* ADCRESULT7 */

#define CSL_ADC_RESULT_ADCRESULT7_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT7_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT7_RESETVAL                                (0x0000U)

/* ADCRESULT8 */

#define CSL_ADC_RESULT_ADCRESULT8_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT8_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT8_RESETVAL                                (0x0000U)

/* ADCRESULT9 */

#define CSL_ADC_RESULT_ADCRESULT9_RESULT_MASK                             (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_SHIFT                            (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_RESETVAL                         (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT9_RESULT_MAX                              (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT9_RESETVAL                                (0x0000U)

/* ADCRESULT10 */

#define CSL_ADC_RESULT_ADCRESULT10_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT10_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT10_RESETVAL                               (0x0000U)

/* ADCRESULT11 */

#define CSL_ADC_RESULT_ADCRESULT11_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT11_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT11_RESETVAL                               (0x0000U)

/* ADCRESULT12 */

#define CSL_ADC_RESULT_ADCRESULT12_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT12_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT12_RESETVAL                               (0x0000U)

/* ADCRESULT13 */

#define CSL_ADC_RESULT_ADCRESULT13_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT13_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT13_RESETVAL                               (0x0000U)

/* ADCRESULT14 */

#define CSL_ADC_RESULT_ADCRESULT14_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT14_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT14_RESETVAL                               (0x0000U)

/* ADCRESULT15 */

#define CSL_ADC_RESULT_ADCRESULT15_RESULT_MASK                            (0xFFFFU)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCRESULT15_RESULT_MAX                             (0xFFFFU)

#define CSL_ADC_RESULT_ADCRESULT15_RESETVAL                               (0x0000U)

/* ADCPPB1RESULT */

#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_MASK                       (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_SHIFT                      (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_RESETVAL                   (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_PPBRESULT_MAX                        (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_MASK                            (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_SHIFT                           (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1RESULT_SIGN_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB1RESULT_RESETVAL                             (0x00000000U)

/* ADCPPB2RESULT */

#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_MASK                       (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_SHIFT                      (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_RESETVAL                   (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_PPBRESULT_MAX                        (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_MASK                            (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_SHIFT                           (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2RESULT_SIGN_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB2RESULT_RESETVAL                             (0x00000000U)

/* ADCPPB3RESULT */

#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_MASK                       (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_SHIFT                      (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_RESETVAL                   (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_PPBRESULT_MAX                        (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_MASK                            (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_SHIFT                           (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3RESULT_SIGN_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB3RESULT_RESETVAL                             (0x00000000U)

/* ADCPPB4RESULT */

#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_MASK                       (0x0000FFFFU)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_SHIFT                      (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_RESETVAL                   (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_PPBRESULT_MAX                        (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_MASK                            (0xFFFF0000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_SHIFT                           (0x00000010U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_RESETVAL                        (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4RESULT_SIGN_MAX                             (0x0000FFFFU)

#define CSL_ADC_RESULT_ADCPPB4RESULT_RESETVAL                             (0x00000000U)

/* ADCPPB1SUM */

#define CSL_ADC_RESULT_ADCPPB1SUM_SUM_MASK                                (0x00FFFFFFU)
#define CSL_ADC_RESULT_ADCPPB1SUM_SUM_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1SUM_SUM_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1SUM_SUM_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_RESULT_ADCPPB1SUM_SIGN_MASK                               (0xFF000000U)
#define CSL_ADC_RESULT_ADCPPB1SUM_SIGN_SHIFT                              (0x00000018U)
#define CSL_ADC_RESULT_ADCPPB1SUM_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1SUM_SIGN_MAX                                (0x000000FFU)

#define CSL_ADC_RESULT_ADCPPB1SUM_RESETVAL                                (0x00000000U)

/* ADCPPB1COUNT */

#define CSL_ADC_RESULT_ADCPPB1COUNT_COUNT_MASK                            (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB1COUNT_COUNT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1COUNT_COUNT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1COUNT_COUNT_MAX                             (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB1COUNT_RESERVED_1_MASK                       (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB1COUNT_RESERVED_1_SHIFT                      (0x000AU)
#define CSL_ADC_RESULT_ADCPPB1COUNT_RESERVED_1_RESETVAL                   (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1COUNT_RESERVED_1_MAX                        (0x003FU)

#define CSL_ADC_RESULT_ADCPPB1COUNT_RESETVAL                              (0x0000U)

/* ADCPPB2SUM */

#define CSL_ADC_RESULT_ADCPPB2SUM_SUM_MASK                                (0x00FFFFFFU)
#define CSL_ADC_RESULT_ADCPPB2SUM_SUM_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2SUM_SUM_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2SUM_SUM_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_RESULT_ADCPPB2SUM_SIGN_MASK                               (0xFF000000U)
#define CSL_ADC_RESULT_ADCPPB2SUM_SIGN_SHIFT                              (0x00000018U)
#define CSL_ADC_RESULT_ADCPPB2SUM_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2SUM_SIGN_MAX                                (0x000000FFU)

#define CSL_ADC_RESULT_ADCPPB2SUM_RESETVAL                                (0x00000000U)

/* ADCPPB2COUNT */

#define CSL_ADC_RESULT_ADCPPB2COUNT_COUNT_MASK                            (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB2COUNT_COUNT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2COUNT_COUNT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2COUNT_COUNT_MAX                             (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB2COUNT_RESERVED_1_MASK                       (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB2COUNT_RESERVED_1_SHIFT                      (0x000AU)
#define CSL_ADC_RESULT_ADCPPB2COUNT_RESERVED_1_RESETVAL                   (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2COUNT_RESERVED_1_MAX                        (0x003FU)

#define CSL_ADC_RESULT_ADCPPB2COUNT_RESETVAL                              (0x0000U)

/* ADCPPB3SUM */

#define CSL_ADC_RESULT_ADCPPB3SUM_SUM_MASK                                (0x00FFFFFFU)
#define CSL_ADC_RESULT_ADCPPB3SUM_SUM_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3SUM_SUM_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3SUM_SUM_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_RESULT_ADCPPB3SUM_SIGN_MASK                               (0xFF000000U)
#define CSL_ADC_RESULT_ADCPPB3SUM_SIGN_SHIFT                              (0x00000018U)
#define CSL_ADC_RESULT_ADCPPB3SUM_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3SUM_SIGN_MAX                                (0x000000FFU)

#define CSL_ADC_RESULT_ADCPPB3SUM_RESETVAL                                (0x00000000U)

/* ADCPPB3COUNT */

#define CSL_ADC_RESULT_ADCPPB3COUNT_COUNT_MASK                            (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB3COUNT_COUNT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3COUNT_COUNT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3COUNT_COUNT_MAX                             (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB3COUNT_RESERVED_1_MASK                       (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB3COUNT_RESERVED_1_SHIFT                      (0x000AU)
#define CSL_ADC_RESULT_ADCPPB3COUNT_RESERVED_1_RESETVAL                   (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3COUNT_RESERVED_1_MAX                        (0x003FU)

#define CSL_ADC_RESULT_ADCPPB3COUNT_RESETVAL                              (0x0000U)

/* ADCPPB4SUM */

#define CSL_ADC_RESULT_ADCPPB4SUM_SUM_MASK                                (0x00FFFFFFU)
#define CSL_ADC_RESULT_ADCPPB4SUM_SUM_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4SUM_SUM_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4SUM_SUM_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_RESULT_ADCPPB4SUM_SIGN_MASK                               (0xFF000000U)
#define CSL_ADC_RESULT_ADCPPB4SUM_SIGN_SHIFT                              (0x00000018U)
#define CSL_ADC_RESULT_ADCPPB4SUM_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4SUM_SIGN_MAX                                (0x000000FFU)

#define CSL_ADC_RESULT_ADCPPB4SUM_RESETVAL                                (0x00000000U)

/* ADCPPB4COUNT */

#define CSL_ADC_RESULT_ADCPPB4COUNT_COUNT_MASK                            (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB4COUNT_COUNT_SHIFT                           (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4COUNT_COUNT_RESETVAL                        (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4COUNT_COUNT_MAX                             (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB4COUNT_RESERVED_1_MASK                       (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB4COUNT_RESERVED_1_SHIFT                      (0x000AU)
#define CSL_ADC_RESULT_ADCPPB4COUNT_RESERVED_1_RESETVAL                   (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4COUNT_RESERVED_1_MAX                        (0x003FU)

#define CSL_ADC_RESULT_ADCPPB4COUNT_RESETVAL                              (0x0000U)

/* ADCPPB1MAX */

#define CSL_ADC_RESULT_ADCPPB1MAX_MAX_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB1MAX_MAX_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MAX_MAX_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MAX_MAX_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB1MAX_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB1MAX_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB1MAX_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MAX_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB1MAX_RESETVAL                                (0x00000000U)

/* ADCPPB1MAXI */

#define CSL_ADC_RESULT_ADCPPB1MAXI_MAXI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB1MAXI_MAXI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MAXI_MAXI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MAXI_MAXI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB1MAXI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB1MAXI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB1MAXI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MAXI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB1MAXI_RESETVAL                               (0x0000U)

/* ADCPPB1MIN */

#define CSL_ADC_RESULT_ADCPPB1MIN_MIN_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB1MIN_MIN_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MIN_MIN_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MIN_MIN_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB1MIN_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB1MIN_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB1MIN_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB1MIN_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB1MIN_RESETVAL                                (0x00000000U)

/* ADCPPB1MINI */

#define CSL_ADC_RESULT_ADCPPB1MINI_MINI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB1MINI_MINI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MINI_MINI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MINI_MINI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB1MINI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB1MINI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB1MINI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB1MINI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB1MINI_RESETVAL                               (0x0000U)

/* ADCPPB2MAX */

#define CSL_ADC_RESULT_ADCPPB2MAX_MAX_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB2MAX_MAX_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MAX_MAX_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MAX_MAX_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB2MAX_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB2MAX_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB2MAX_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MAX_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB2MAX_RESETVAL                                (0x00000000U)

/* ADCPPB2MAXI */

#define CSL_ADC_RESULT_ADCPPB2MAXI_MAXI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB2MAXI_MAXI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MAXI_MAXI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MAXI_MAXI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB2MAXI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB2MAXI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB2MAXI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MAXI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB2MAXI_RESETVAL                               (0x0000U)

/* ADCPPB2MIN */

#define CSL_ADC_RESULT_ADCPPB2MIN_MIN_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB2MIN_MIN_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MIN_MIN_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MIN_MIN_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB2MIN_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB2MIN_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB2MIN_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB2MIN_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB2MIN_RESETVAL                                (0x00000000U)

/* ADCPPB2MINI */

#define CSL_ADC_RESULT_ADCPPB2MINI_MINI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB2MINI_MINI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MINI_MINI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MINI_MINI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB2MINI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB2MINI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB2MINI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB2MINI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB2MINI_RESETVAL                               (0x0000U)

/* ADCPPB3MAX */

#define CSL_ADC_RESULT_ADCPPB3MAX_MAX_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB3MAX_MAX_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MAX_MAX_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MAX_MAX_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB3MAX_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB3MAX_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB3MAX_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MAX_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB3MAX_RESETVAL                                (0x00000000U)

/* ADCPPB3MAXI */

#define CSL_ADC_RESULT_ADCPPB3MAXI_MAXI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB3MAXI_MAXI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MAXI_MAXI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MAXI_MAXI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB3MAXI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB3MAXI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB3MAXI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MAXI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB3MAXI_RESETVAL                               (0x0000U)

/* ADCPPB3MIN */

#define CSL_ADC_RESULT_ADCPPB3MIN_MIN_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB3MIN_MIN_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MIN_MIN_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MIN_MIN_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB3MIN_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB3MIN_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB3MIN_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB3MIN_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB3MIN_RESETVAL                                (0x00000000U)

/* ADCPPB3MINI */

#define CSL_ADC_RESULT_ADCPPB3MINI_MINI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB3MINI_MINI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MINI_MINI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MINI_MINI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB3MINI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB3MINI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB3MINI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB3MINI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB3MINI_RESETVAL                               (0x0000U)

/* ADCPPB4MAX */

#define CSL_ADC_RESULT_ADCPPB4MAX_MAX_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB4MAX_MAX_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MAX_MAX_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MAX_MAX_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB4MAX_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB4MAX_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB4MAX_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MAX_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB4MAX_RESETVAL                                (0x00000000U)

/* ADCPPB4MAXI */

#define CSL_ADC_RESULT_ADCPPB4MAXI_MAXI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB4MAXI_MAXI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MAXI_MAXI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MAXI_MAXI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB4MAXI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB4MAXI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB4MAXI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MAXI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB4MAXI_RESETVAL                               (0x0000U)

/* ADCPPB4MIN */

#define CSL_ADC_RESULT_ADCPPB4MIN_MIN_MASK                                (0x0001FFFFU)
#define CSL_ADC_RESULT_ADCPPB4MIN_MIN_SHIFT                               (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MIN_MIN_RESETVAL                            (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MIN_MIN_MAX                                 (0x0001FFFFU)

#define CSL_ADC_RESULT_ADCPPB4MIN_SIGN_MASK                               (0xFFFE0000U)
#define CSL_ADC_RESULT_ADCPPB4MIN_SIGN_SHIFT                              (0x00000011U)
#define CSL_ADC_RESULT_ADCPPB4MIN_SIGN_RESETVAL                           (0x00000000U)
#define CSL_ADC_RESULT_ADCPPB4MIN_SIGN_MAX                                (0x00007FFFU)

#define CSL_ADC_RESULT_ADCPPB4MIN_RESETVAL                                (0x00000000U)

/* ADCPPB4MINI */

#define CSL_ADC_RESULT_ADCPPB4MINI_MINI_MASK                              (0x03FFU)
#define CSL_ADC_RESULT_ADCPPB4MINI_MINI_SHIFT                             (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MINI_MINI_RESETVAL                          (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MINI_MINI_MAX                               (0x03FFU)

#define CSL_ADC_RESULT_ADCPPB4MINI_RESERVED_1_MASK                        (0xFC00U)
#define CSL_ADC_RESULT_ADCPPB4MINI_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_ADC_RESULT_ADCPPB4MINI_RESERVED_1_RESETVAL                    (0x0000U)
#define CSL_ADC_RESULT_ADCPPB4MINI_RESERVED_1_MAX                         (0x003FU)

#define CSL_ADC_RESULT_ADCPPB4MINI_RESETVAL                               (0x0000U)

#ifdef __cplusplus
}
#endif
#endif
