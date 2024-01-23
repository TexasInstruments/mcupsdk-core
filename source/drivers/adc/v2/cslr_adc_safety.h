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
 *  Name        : cslr_adc_safety.h
*/
#ifndef CSLR_ADC_SAFETY_H_
#define CSLR_ADC_SAFETY_H_

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
    volatile uint16_t CHECKCONFIG;
    volatile uint8_t  Resv_4[2];
    volatile uint16_t CHECKSTATUS;
    volatile uint8_t  Resv_8[2];
    volatile uint16_t ADCRESSEL1;
    volatile uint8_t  Resv_12[2];
    volatile uint16_t ADCRESSEL2;
    volatile uint8_t  Resv_16[2];
    volatile uint32_t TOLERANCE;
    volatile uint8_t  Resv_24[4];
    volatile uint32_t CHECKRESULT1;
    volatile uint32_t CHECKRESULT2;
} CSL_adc_safetyRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ADC_SAFETY_CHECKCONFIG                                             (0x00000000U)
#define CSL_ADC_SAFETY_CHECKSTATUS                                             (0x00000004U)
#define CSL_ADC_SAFETY_ADCRESSEL1                                              (0x00000008U)
#define CSL_ADC_SAFETY_ADCRESSEL2                                              (0x0000000CU)
#define CSL_ADC_SAFETY_TOLERANCE                                               (0x00000010U)
#define CSL_ADC_SAFETY_CHECKRESULT1                                            (0x00000018U)
#define CSL_ADC_SAFETY_CHECKRESULT2                                            (0x0000001CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CHECKCONFIG */

#define CSL_ADC_SAFETY_CHECKCONFIG_SYNCINSEL_MASK                              (0x001FU)
#define CSL_ADC_SAFETY_CHECKCONFIG_SYNCINSEL_SHIFT                             (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_SYNCINSEL_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_SYNCINSEL_MAX                               (0x001FU)

#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_1_MASK                             (0x0020U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_1_SHIFT                            (0x0005U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_1_MAX                              (0x0001U)

#define CSL_ADC_SAFETY_CHECKCONFIG_SWSYNC_MASK                                 (0x0040U)
#define CSL_ADC_SAFETY_CHECKCONFIG_SWSYNC_SHIFT                                (0x0006U)
#define CSL_ADC_SAFETY_CHECKCONFIG_SWSYNC_RESETVAL                             (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_SWSYNC_MAX                                  (0x0001U)

#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_2_MASK                             (0x7F80U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_2_SHIFT                            (0x0007U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_RESERVED_2_MAX                              (0x00FFU)

#define CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK                                  (0x8000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_SHIFT                                 (0x000FU)
#define CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_RESETVAL                              (0x0000U)
#define CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MAX                                   (0x0001U)

#define CSL_ADC_SAFETY_CHECKCONFIG_RESETVAL                                    (0x0000U)

/* CHECKSTATUS */

#define CSL_ADC_SAFETY_CHECKSTATUS_RES1READY_MASK                              (0x0001U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES1READY_SHIFT                             (0x0000U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES1READY_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES1READY_MAX                               (0x0001U)

#define CSL_ADC_SAFETY_CHECKSTATUS_RES2READY_MASK                              (0x0002U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES2READY_SHIFT                             (0x0001U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES2READY_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RES2READY_MAX                               (0x0001U)

#define CSL_ADC_SAFETY_CHECKSTATUS_OOT_MASK                                    (0x0004U)
#define CSL_ADC_SAFETY_CHECKSTATUS_OOT_SHIFT                                   (0x0002U)
#define CSL_ADC_SAFETY_CHECKSTATUS_OOT_RESETVAL                                (0x0000U)
#define CSL_ADC_SAFETY_CHECKSTATUS_OOT_MAX                                     (0x0001U)

#define CSL_ADC_SAFETY_CHECKSTATUS_RESERVED_1_MASK                             (0xFFF8U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RESERVED_1_SHIFT                            (0x0003U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_ADC_SAFETY_CHECKSTATUS_RESERVED_1_MAX                              (0x1FFFU)

#define CSL_ADC_SAFETY_CHECKSTATUS_RESETVAL                                    (0x0000U)

/* ADCRESSEL1 */

#define CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_MASK                                  (0x0007U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_SHIFT                                 (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_RESETVAL                              (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_MAX                                   (0x0007U)

#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_1_MASK                              (0x0008U)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_1_SHIFT                             (0x0003U)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_1_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_1_MAX                               (0x0001U)

#define CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_MASK                            (0x03F0U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_SHIFT                           (0x0004U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_RESETVAL                        (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_MAX                             (0x003FU)

#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_2_MASK                              (0xFC00U)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_2_SHIFT                             (0x000AU)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_2_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL1_RESERVED_2_MAX                               (0x003FU)

#define CSL_ADC_SAFETY_ADCRESSEL1_RESETVAL                                     (0x0000U)

/* ADCRESSEL2 */

#define CSL_ADC_SAFETY_ADCRESSEL2_ADCSEL_MASK                                  (0x0007U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCSEL_SHIFT                                 (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCSEL_RESETVAL                              (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCSEL_MAX                                   (0x0007U)

#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_1_MASK                              (0x0008U)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_1_SHIFT                             (0x0003U)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_1_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_1_MAX                               (0x0001U)

#define CSL_ADC_SAFETY_ADCRESSEL2_ADCRESULTSEL_MASK                            (0x03F0U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCRESULTSEL_SHIFT                           (0x0004U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCRESULTSEL_RESETVAL                        (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL2_ADCRESULTSEL_MAX                             (0x003FU)

#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_2_MASK                              (0xFC00U)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_2_SHIFT                             (0x000AU)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_2_RESETVAL                          (0x0000U)
#define CSL_ADC_SAFETY_ADCRESSEL2_RESERVED_2_MAX                               (0x003FU)

#define CSL_ADC_SAFETY_ADCRESSEL2_RESETVAL                                     (0x0000U)

/* TOLERANCE */

#define CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MASK                                (0x00FFFFFFU)
#define CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_SHIFT                               (0x00000000U)
#define CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_SAFETY_TOLERANCE_RESERVED_1_MASK                               (0xFF000000U)
#define CSL_ADC_SAFETY_TOLERANCE_RESERVED_1_SHIFT                              (0x00000018U)
#define CSL_ADC_SAFETY_TOLERANCE_RESERVED_1_RESETVAL                           (0x00000000U)
#define CSL_ADC_SAFETY_TOLERANCE_RESERVED_1_MAX                                (0x000000FFU)

#define CSL_ADC_SAFETY_TOLERANCE_RESETVAL                                      (0x00000000U)

/* CHECKRESULT1 */

#define CSL_ADC_SAFETY_CHECKRESULT1_RESULT_MASK                                (0x00FFFFFFU)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESULT_SHIFT                               (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESULT_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESULT_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_SAFETY_CHECKRESULT1_RESERVED_1_MASK                            (0xFF000000U)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESERVED_1_SHIFT                           (0x00000018U)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT1_RESERVED_1_MAX                             (0x000000FFU)

#define CSL_ADC_SAFETY_CHECKRESULT1_RESETVAL                                   (0x00000000U)

/* CHECKRESULT2 */

#define CSL_ADC_SAFETY_CHECKRESULT2_RESULT_MASK                                (0x00FFFFFFU)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESULT_SHIFT                               (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESULT_RESETVAL                            (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESULT_MAX                                 (0x00FFFFFFU)

#define CSL_ADC_SAFETY_CHECKRESULT2_RESERVED_1_MASK                            (0xFF000000U)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESERVED_1_SHIFT                           (0x00000018U)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESERVED_1_RESETVAL                        (0x00000000U)
#define CSL_ADC_SAFETY_CHECKRESULT2_RESERVED_1_MAX                             (0x000000FFU)

#define CSL_ADC_SAFETY_CHECKRESULT2_RESETVAL                                   (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
