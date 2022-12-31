/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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
 */

#ifndef HW_ADC_H_
#define HW_ADC_H_

#ifdef __cplusplus
extern "C"
{
#endif


/****************************************************************************************************
* Register Definitions
****************************************************************************************************/

#define ADC_REVISION                                                (0x0U)
#define ADC_SYSCONFIG                                               (0x10U)
#define ADC_IRQSTATUS_RAW                                           (0x24U)
#define ADC_IRQSTATUS                                               (0x28U)
#define ADC_IRQENABLE_SET                                           (0x2cU)
#define ADC_IRQENABLE_CLR                                           (0x30U)
#define ADC_IRQWAKEUP                                               (0x34U)
#define ADC_DMAENABLE_SET                                           (0x38U)
#define ADC_DMAENABLE_CLR                                           (0x3cU)
#define ADC_CTRL                                                    (0x40U)
#define ADC_ADCSTAT                                                 (0x44U)
#define ADC_ADCRANGE                                                (0x48U)
#define ADC_CLKDIV                                                  (0x4cU)
#define ADC_MISC                                                    (0x50U)
#define ADC_STEPENABLE                                              (0x54U)
#define ADC_IDLECONFIG                                              (0x58U)
#define ADC_TS_CHARGE_STEPCONFIG                                    (0x5cU)
#define ADC_TS_CHARGE_DELAY                                         (0x60U)
#define ADC_IRQ_EOI                                                 (0x20U)
#define ADC_STEPCONFIG(m)                                           ((uint32_t)0x64U + ((m) * 0x8U))
#define ADC_STEPCONFIG_NUM_ELEMS                                    (16U)
#define ADC_STEPDELAY(m)                                            ((uint32_t)0x68U + ((m) * 0x8U))
#define ADC_STEPDELAY_NUM_ELEMS                                     (16U)
#define ADC_FIFOCOUNT(m)                                            ((uint32_t)0xe4U + ((m) * 0xcU))
#define ADC_FIFOCOUNT_NUM_ELEMS                                     (2U)
#define ADC_FIFOTHRESHOLD(m)                                        ((uint32_t)0xe8U + ((m) * 0xcU))
#define ADC_FIFOTHRESHOLD_NUM_ELEMS                                 (2U)
#define ADC_DMAREQ(m)                                               ((uint32_t)0xecU + ((m) * 0xcU))
#define ADC_DMAREQ_NUM_ELEMS                                        (2U)
#define ADC_FIFODATA(m)                                             ((uint32_t)0x100U + ((m) * 0x100U))
#define ADC_FIFODATA_NUM_ELEMS                                      (2U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

#define ADC_REVISION_X_MAJOR_SHIFT                                                      (8U)
#define ADC_REVISION_X_MAJOR_MASK                                                       (0x00000700U)

#define ADC_REVISION_FUNC_SHIFT                                                         (16U)
#define ADC_REVISION_FUNC_MASK                                                          (0x0fff0000U)

#define ADC_REVISION_R_RTL_SHIFT                                                        (11U)
#define ADC_REVISION_R_RTL_MASK                                                         (0x0000f800U)

#define ADC_REVISION_RESERVED0_SHIFT                                                     (28U)
#define ADC_REVISION_RESERVED0_MASK                                                      (0x30000000U)

#define ADC_REVISION_SCHEME_SHIFT                                                       (30U)
#define ADC_REVISION_SCHEME_MASK                                                        (0xc0000000U)

#define ADC_REVISION_CUSTOM_SHIFT                                                       (6U)
#define ADC_REVISION_CUSTOM_MASK                                                        (0x000000c0U)

#define ADC_REVISION_Y_MINOR_SHIFT                                                      (0U)
#define ADC_REVISION_Y_MINOR_MASK                                                       (0x0000003fU)

#define ADC_IRQSTATUS_RAW_FIFO0_THR_SHIFT                                               (2U)
#define ADC_IRQSTATUS_RAW_FIFO0_THR_MASK                                                (0x00000004U)
#define ADC_IRQSTATUS_RAW_FIFO0_THR_EVT_PEND                                             (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_THR_NO_EVT_PEND                                          (0U)
#define ADC_IRQSTATUS_RAW_FIFO0_THR_SET_EVT                                              (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_THR_NO_ACTION                                            (0U)

#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_SHIFT                                         (7U)
#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_MASK                                          (0x00000080U)
#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_EVT_PEND                                       (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_NO_EVT_PEND                                    (0U)
#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_SET_EVT                                        (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_SHIFT                                         (1U)
#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_MASK                                          (0x00000002U)
#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_EVT_PEND                                       (1U)
#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_NO_EVT_PEND                                    (0U)
#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_SET_EVT                                        (1U)
#define ADC_IRQSTATUS_RAW_END_OF_SEQUENCE_NO_ACTION                                      (0U)

#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_SHIFT                                         (0U)
#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_MASK                                          (0x00000001U)
#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_EVT_PEND                                       (1U)
#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_NO_EVT_PEND                                    (0U)
#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_SET_EVT                                        (1U)
#define ADC_IRQSTATUS_RAW_AFE_EOC_MISSING_NO_ACTION                                      (0U)

#define ADC_IRQSTATUS_RAW_RESERVED0_SHIFT                                                (11U)
#define ADC_IRQSTATUS_RAW_RESERVED0_MASK                                                 (0xfffff800U)

#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_SHIFT                                 (0U)
#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_MASK                                  (0x00000001U)
#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_PEND                                   (1U)
#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_NO_PEND                                (0U)
#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_SET                                    (1U)
#define ADC_IRQSTATUS_RAW_HW_PEN_EVT_ASYNCHRONOUS_NO_ACTION                              (0U)

#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_SHIFT                                           (6U)
#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_MASK                                            (0x00000040U)
#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_EVT_PEND                                         (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_NO_EVT_PEND                                      (0U)
#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_SET_EVT                                          (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_SHIFT                                         (4U)
#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_MASK                                          (0x00000010U)
#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_EVT_PEND                                       (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_NO_EVT_PEND                                    (0U)
#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_SET_EVT                                        (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_SHIFT                                           (3U)
#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_MASK                                            (0x00000008U)
#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_EVT_PEND                                         (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_NO_EVT_PEND                                      (0U)
#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_SET_EVT                                          (1U)
#define ADC_IRQSTATUS_RAW_FIFO0_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_SHIFT                                            (8U)
#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_MASK                                             (0x00000100U)
#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_EVT_PEND                                          (1U)
#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_NO_EVT_PEND                                       (0U)
#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_SET_EVT                                           (1U)
#define ADC_IRQSTATUS_RAW_OUT_OF_RANGE_NO_ACTION                                         (0U)

#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_SHIFT                                    (10U)
#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_MASK                                     (0x00000400U)
#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_EVT_PEND                                  (1U)
#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_NO_EVT_PEND                               (0U)
#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_SET_EVT                                   (1U)
#define ADC_IRQSTATUS_RAW_PEN_IRQ_SYNCHRONIZED_NO_ACTION                                 (0U)

#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_SHIFT                                              (9U)
#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_MASK                                               (0x00000200U)
#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_PEND                                                (1U)
#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_NO_PEND                                             (0U)
#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_SET                                                 (1U)
#define ADC_IRQSTATUS_RAW_PEN_UP_EVT_NO_ACTION                                           (0U)

#define ADC_IRQSTATUS_RAW_FIFO1_THR_SHIFT                                               (5U)
#define ADC_IRQSTATUS_RAW_FIFO1_THR_MASK                                                (0x00000020U)
#define ADC_IRQSTATUS_RAW_FIFO1_THR_EVT_PEND                                             (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_THR_NO_EVT_PEND                                          (0U)
#define ADC_IRQSTATUS_RAW_FIFO1_THR_SET_EVT                                              (1U)
#define ADC_IRQSTATUS_RAW_FIFO1_THR_NO_ACTION                                            (0U)

#define ADC_IRQSTATUS_RESERVED0_SHIFT                                                    (11U)
#define ADC_IRQSTATUS_RESERVED0_MASK                                                     (0xfffff800U)

#define ADC_IRQSTATUS_END_OF_SEQUENCE_SHIFT                                             (1U)
#define ADC_IRQSTATUS_END_OF_SEQUENCE_MASK                                              (0x00000002U)
#define ADC_IRQSTATUS_END_OF_SEQUENCE_EVT_PEND                                           (1U)
#define ADC_IRQSTATUS_END_OF_SEQUENCE_NO_EVT_PEND                                        (0U)
#define ADC_IRQSTATUS_END_OF_SEQUENCE_CLR_EVT                                            (1U)
#define ADC_IRQSTATUS_END_OF_SEQUENCE_NO_ACTION                                          (0U)

#define ADC_IRQSTATUS_AFE_EOC_MISSING_SHIFT                                             (0U)
#define ADC_IRQSTATUS_AFE_EOC_MISSING_MASK                                              (0x00000001U)
#define ADC_IRQSTATUS_AFE_EOC_MISSING_EVT_PEND                                           (1U)
#define ADC_IRQSTATUS_AFE_EOC_MISSING_NO_EVT_PEND                                        (0U)
#define ADC_IRQSTATUS_AFE_EOC_MISSING_SET_EVT                                            (1U)
#define ADC_IRQSTATUS_AFE_EOC_MISSING_NO_ACTION                                          (0U)

#define ADC_IRQSTATUS_PEN_UP_EVT_SHIFT                                                  (9U)
#define ADC_IRQSTATUS_PEN_UP_EVT_MASK                                                   (0x00000200U)
#define ADC_IRQSTATUS_PEN_UP_EVT_PEND                                                    (1U)
#define ADC_IRQSTATUS_PEN_UP_EVT_NO_PEND                                                 (0U)
#define ADC_IRQSTATUS_PEN_UP_EVT_CLR                                                     (1U)
#define ADC_IRQSTATUS_PEN_UP_EVT_NO_ACTION                                               (0U)

#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_SHIFT                                             (7U)
#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_MASK                                              (0x00000080U)
#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_EVT_PEND                                           (1U)
#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_NO_EVT_PEND                                        (0U)
#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_CLR_EVT                                            (1U)
#define ADC_IRQSTATUS_FIFO1_UNDERFLOW_NO_ACTION                                          (0U)

#define ADC_IRQSTATUS_FIFO1_THR_SHIFT                                                   (5U)
#define ADC_IRQSTATUS_FIFO1_THR_MASK                                                    (0x00000020U)
#define ADC_IRQSTATUS_FIFO1_THR_EVT_PEND                                                 (1U)
#define ADC_IRQSTATUS_FIFO1_THR_NO_EVT_PEND                                              (0U)
#define ADC_IRQSTATUS_FIFO1_THR_CLR_EVT                                                  (1U)
#define ADC_IRQSTATUS_FIFO1_THR_NO_ACTION                                                (0U)

#define ADC_IRQSTATUS_FIFO1_OVERRUN_SHIFT                                               (6U)
#define ADC_IRQSTATUS_FIFO1_OVERRUN_MASK                                                (0x00000040U)
#define ADC_IRQSTATUS_FIFO1_OVERRUN_EVT_PEND                                             (1U)
#define ADC_IRQSTATUS_FIFO1_OVERRUN_NO_EVT_PEND                                          (0U)
#define ADC_IRQSTATUS_FIFO1_OVERRUN_CLR_EVT                                              (1U)
#define ADC_IRQSTATUS_FIFO1_OVERRUN_NO_ACTION                                            (0U)

#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_SHIFT                                             (4U)
#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_MASK                                              (0x00000010U)
#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_EVT_PEND                                           (1U)
#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_NO_EVT_PEND                                        (0U)
#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_CLR_EVT                                            (1U)
#define ADC_IRQSTATUS_FIFO0_UNDERFLOW_NO_ACTION                                          (0U)

#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_SHIFT                                     (0U)
#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_MASK                                      (0x00000001U)
#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_PEND                                       (1U)
#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_NO_PEND                                    (0U)
#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_CLR                                        (1U)
#define ADC_IRQSTATUS_HW_PEN_EVT_ASYNCHRONOUS_NO_ACTION                                  (0U)

#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_SHIFT                                      (10U)
#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_MASK                                       (0x00000400U)
#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_PEND                                        (1U)
#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_NO_PEND                                     (0U)
#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_CLR                                         (1U)
#define ADC_IRQSTATUS_HW_PEN_EVT_SYNCHRONOUS_NO_ACTION                                   (0U)

#define ADC_IRQSTATUS_FIFO0_OVERRUN_SHIFT                                               (3U)
#define ADC_IRQSTATUS_FIFO0_OVERRUN_MASK                                                (0x00000008U)
#define ADC_IRQSTATUS_FIFO0_OVERRUN_EVT_PEND                                             (1U)
#define ADC_IRQSTATUS_FIFO0_OVERRUN_NO_EVT_PEND                                          (0U)
#define ADC_IRQSTATUS_FIFO0_OVERRUN_CLR_EVT                                              (1U)
#define ADC_IRQSTATUS_FIFO0_OVERRUN_NO_ACTION                                            (0U)

#define ADC_IRQSTATUS_OUT_OF_RANGE_SHIFT                                                (8U)
#define ADC_IRQSTATUS_OUT_OF_RANGE_MASK                                                 (0x00000100U)
#define ADC_IRQSTATUS_OUT_OF_RANGE_EVT_PEND                                              (1U)
#define ADC_IRQSTATUS_OUT_OF_RANGE_NO_EVT_PEND                                           (0U)
#define ADC_IRQSTATUS_OUT_OF_RANGE_CLR_EVT                                               (1U)
#define ADC_IRQSTATUS_OUT_OF_RANGE_NO_ACTION                                             (0U)

#define ADC_IRQSTATUS_FIFO0_THR_SHIFT                                                   (2U)
#define ADC_IRQSTATUS_FIFO0_THR_MASK                                                    (0x00000004U)
#define ADC_IRQSTATUS_FIFO0_THR_EVT_PEND                                                 (1U)
#define ADC_IRQSTATUS_FIFO0_THR_NO_EVT_PEND                                              (0U)
#define ADC_IRQSTATUS_FIFO0_THR_CLR_EVT                                                  (1U)
#define ADC_IRQSTATUS_FIFO0_THR_NO_ACTION                                                (0U)

#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_SHIFT                                 (0U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_MASK                                  (0x00000001U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_ENABLED                                (1U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_DISABLED                               (0U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_ENABLE                                 (1U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_ASYNCHRONOUS_NO_ACTION                              (0U)

#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_SHIFT                                         (4U)
#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_MASK                                          (0x00000010U)
#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_ENABLED                                        (1U)
#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_DISABLED                                       (0U)
#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_ENABLE                                         (1U)
#define ADC_IRQENABLE_SET_FIFO0_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_SHIFT                                           (6U)
#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_MASK                                            (0x00000040U)
#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_ENABLED                                          (1U)
#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_DISABLED                                         (0U)
#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_ENABLE                                           (1U)
#define ADC_IRQENABLE_SET_FIFO1_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_SHIFT                                           (3U)
#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_MASK                                            (0x00000008U)
#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_ENABLED                                          (1U)
#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_DISABLED                                         (0U)
#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_ENABLE                                           (1U)
#define ADC_IRQENABLE_SET_FIFO0_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQENABLE_SET_FIFO1_THR_SHIFT                                               (5U)
#define ADC_IRQENABLE_SET_FIFO1_THR_MASK                                                (0x00000020U)
#define ADC_IRQENABLE_SET_FIFO1_THR_ENABLED                                              (1U)
#define ADC_IRQENABLE_SET_FIFO1_THR_DISABLED                                             (0U)
#define ADC_IRQENABLE_SET_FIFO1_THR_ENABLE                                               (1U)
#define ADC_IRQENABLE_SET_FIFO1_THR_NO_ACTION                                            (0U)

#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_SHIFT                                         (7U)
#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_MASK                                          (0x00000080U)
#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_ENABLED                                        (1U)
#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_DISABLED                                       (0U)
#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_ENABLE                                         (1U)
#define ADC_IRQENABLE_SET_FIFO1_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_SET_FIFO0_THR_SHIFT                                               (2U)
#define ADC_IRQENABLE_SET_FIFO0_THR_MASK                                                (0x00000004U)
#define ADC_IRQENABLE_SET_FIFO0_THR_ENABLED                                              (1U)
#define ADC_IRQENABLE_SET_FIFO0_THR_DISABLED                                             (0U)
#define ADC_IRQENABLE_SET_FIFO0_THR_ENABLE                                               (1U)
#define ADC_IRQENABLE_SET_FIFO0_THR_NO_ACTION                                            (0U)

#define ADC_IRQENABLE_SET_RESERVED0_SHIFT                                                (11U)
#define ADC_IRQENABLE_SET_RESERVED0_MASK                                                 (0xfffff800U)

#define ADC_IRQENABLE_SET_PEN_UP_EVT_SHIFT                                              (9U)
#define ADC_IRQENABLE_SET_PEN_UP_EVT_MASK                                               (0x00000200U)
#define ADC_IRQENABLE_SET_PEN_UP_EVT_ENABLED                                             (1U)
#define ADC_IRQENABLE_SET_PEN_UP_EVT_DISABLED                                            (0U)
#define ADC_IRQENABLE_SET_PEN_UP_EVT_ENABLE                                              (1U)
#define ADC_IRQENABLE_SET_PEN_UP_EVT_NO_ACTION                                           (0U)

#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_SHIFT                                         (1U)
#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_MASK                                          (0x00000002U)
#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_ENABLED                                        (1U)
#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_DISABLED                                       (0U)
#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_ENABLE                                         (1U)
#define ADC_IRQENABLE_SET_END_OF_SEQUENCE_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_SHIFT                                         (0U)
#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_MASK                                          (0x00000001U)
#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_EVT_PEND                                       (1U)
#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_NO_EVT_PEND                                    (0U)
#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_SET_EVT                                        (1U)
#define ADC_IRQENABLE_SET_AFE_EOC_MISSING_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_SET_OUT_OF_RANGE_SHIFT                                            (8U)
#define ADC_IRQENABLE_SET_OUT_OF_RANGE_MASK                                             (0x00000100U)
#define ADC_IRQENABLE_SET_OUT_OF_RANGE_ENABLED                                           (1U)
#define ADC_IRQENABLE_SET_OUT_OF_RANGE_DISABLED                                          (0U)
#define ADC_IRQENABLE_SET_OUT_OF_RANGE_ENABLE                                            (1U)
#define ADC_IRQENABLE_SET_OUT_OF_RANGE_NO_ACTION                                         (0U)

#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_SHIFT                                  (10U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_MASK                                   (0x00000400U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_ENABLED                                 (1U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_DISABLED                                (0U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_ENABLE                                  (1U)
#define ADC_IRQENABLE_SET_HW_PEN_EVT_SYNCHRONOUS_NO_ACTION                               (0U)

#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_SHIFT                                           (3U)
#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_MASK                                            (0x00000008U)
#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_ENABLED                                          (1U)
#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_DISABLED                                         (0U)
#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_DISABLE                                          (1U)
#define ADC_IRQENABLE_CLR_FIFO0_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_SHIFT                                 (0U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_MASK                                  (0x00000001U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_ENABLED                                (1U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_DISABLED                               (0U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_DISABLE                                (1U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_ASYNCHRONOUS_NO_ACTION                              (0U)

#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_SHIFT                                  (10U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_MASK                                   (0x00000400U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_ENABLED                                 (1U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_DISABLED                                (0U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_DISABLE                                 (1U)
#define ADC_IRQENABLE_CLR_HW_PEN_EVT_SYNCHRONOUS_NO_ACTION                               (0U)

#define ADC_IRQENABLE_CLR_FIFO1_THR_SHIFT                                               (5U)
#define ADC_IRQENABLE_CLR_FIFO1_THR_MASK                                                (0x00000020U)
#define ADC_IRQENABLE_CLR_FIFO1_THR_ENABLED                                              (1U)
#define ADC_IRQENABLE_CLR_FIFO1_THR_DISABLED                                             (0U)
#define ADC_IRQENABLE_CLR_FIFO1_THR_DISABLE                                              (1U)
#define ADC_IRQENABLE_CLR_FIFO1_THR_NO_ACTION                                            (0U)

#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_SHIFT                                         (4U)
#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_MASK                                          (0x00000010U)
#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_ENABLED                                        (1U)
#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_DISABLED                                       (0U)
#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_DISABLE                                        (1U)
#define ADC_IRQENABLE_CLR_FIFO0_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_CLR_PEN_UP_EVT_SHIFT                                              (9U)
#define ADC_IRQENABLE_CLR_PEN_UP_EVT_MASK                                               (0x00000200U)
#define ADC_IRQENABLE_CLR_PEN_UP_EVT_ENABLED                                             (1U)
#define ADC_IRQENABLE_CLR_PEN_UP_EVT_DISABLED                                            (0U)
#define ADC_IRQENABLE_CLR_PEN_UP_EVT_DISABLE                                             (1U)
#define ADC_IRQENABLE_CLR_PEN_UP_EVT_NO_ACTION                                           (0U)

#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_SHIFT                                         (1U)
#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_MASK                                          (0x00000002U)
#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_ENABLED                                        (1U)
#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_DISABLED                                       (0U)
#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_DISABLE                                        (1U)
#define ADC_IRQENABLE_CLR_END_OF_SEQUENCE_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_SHIFT                                         (0U)
#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_MASK                                          (0x00000001U)
#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_EVT_PEND                                       (1U)
#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_NO_EVT_PEND                                    (0U)
#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_SET_EVT                                        (1U)
#define ADC_IRQENABLE_CLR_AFE_EOC_MISSING_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_CLR_RESERVED0_SHIFT                                                (11U)
#define ADC_IRQENABLE_CLR_RESERVED0_MASK                                                 (0xfffff800U)

#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_SHIFT                                         (7U)
#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_MASK                                          (0x00000080U)
#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_ENABLED                                        (1U)
#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_DISABLED                                       (0U)
#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_DISABLE                                        (1U)
#define ADC_IRQENABLE_CLR_FIFO1_UNDERFLOW_NO_ACTION                                      (0U)

#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_SHIFT                                            (8U)
#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_MASK                                             (0x00000100U)
#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_ENABLED                                           (1U)
#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_DISABLED                                          (0U)
#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_DISABLE                                           (1U)
#define ADC_IRQENABLE_CLR_OUT_OF_RANGE_NO_ACTION                                         (0U)

#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_SHIFT                                           (6U)
#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_MASK                                            (0x00000040U)
#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_ENABLED                                          (1U)
#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_DISABLED                                         (0U)
#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_DISABLE                                          (1U)
#define ADC_IRQENABLE_CLR_FIFO1_OVERRUN_NO_ACTION                                        (0U)

#define ADC_IRQENABLE_CLR_FIFO0_THR_SHIFT                                               (2U)
#define ADC_IRQENABLE_CLR_FIFO0_THR_MASK                                                (0x00000004U)
#define ADC_IRQENABLE_CLR_FIFO0_THR_ENABLED                                              (1U)
#define ADC_IRQENABLE_CLR_FIFO0_THR_DISABLED                                             (0U)
#define ADC_IRQENABLE_CLR_FIFO0_THR_DISABLE                                              (1U)
#define ADC_IRQENABLE_CLR_FIFO0_THR_NO_ACTION                                            (0U)

#define ADC_IRQWAKEUP_WAKEEN0_SHIFT                                                     (0U)
#define ADC_IRQWAKEUP_WAKEEN0_MASK                                                      (0x00000001U)
#define ADC_IRQWAKEUP_WAKEEN0_DISABLED                                                   (0U)
#define ADC_IRQWAKEUP_WAKEEN0_ENABLED                                                    (1U)

#define ADC_IRQWAKEUP_RESERVED0_SHIFT                                                    (1U)
#define ADC_IRQWAKEUP_RESERVED0_MASK                                                     (0xfffffffeU)

#define ADC_DMAENABLE_SET_EN_0_SHIFT                                                    (0U)
#define ADC_DMAENABLE_SET_EN_0_MASK                                                     (0x00000001U)
#define ADC_DMAENABLE_SET_EN_0_ENABLED                                                   (1U)
#define ADC_DMAENABLE_SET_EN_0_DISABLED                                                  (0U)
#define ADC_DMAENABLE_SET_EN_0_ENABLE                                                    (1U)
#define ADC_DMAENABLE_SET_EN_0_NO_ACTION                                                 (0U)

#define ADC_DMAENABLE_SET_RESERVED0_SHIFT                                                (2U)
#define ADC_DMAENABLE_SET_RESERVED0_MASK                                                 (0xfffffffcU)

#define ADC_DMAENABLE_SET_EN_1_SHIFT                                                    (1U)
#define ADC_DMAENABLE_SET_EN_1_MASK                                                     (0x00000002U)
#define ADC_DMAENABLE_SET_EN_1_ENABLED                                                   (1U)
#define ADC_DMAENABLE_SET_EN_1_DISABLED                                                  (0U)
#define ADC_DMAENABLE_SET_EN_1_ENABLE                                                    (1U)
#define ADC_DMAENABLE_SET_EN_1_NO_ACTION                                                 (0U)

#define ADC_DMAENABLE_CLR_RESERVED0_SHIFT                                                (2U)
#define ADC_DMAENABLE_CLR_RESERVED0_MASK                                                 (0xfffffffcU)

#define ADC_DMAENABLE_CLR_EN_1_SHIFT                                                    (1U)
#define ADC_DMAENABLE_CLR_EN_1_MASK                                                     (0x00000002U)
#define ADC_DMAENABLE_CLR_EN_1_ENABLED                                                   (1U)
#define ADC_DMAENABLE_CLR_EN_1_DISABLED                                                  (0U)
#define ADC_DMAENABLE_CLR_EN_1_DISABLE                                                   (1U)
#define ADC_DMAENABLE_CLR_EN_1_NO_ACTION                                                 (0U)

#define ADC_DMAENABLE_CLR_EN_0_SHIFT                                                    (0U)
#define ADC_DMAENABLE_CLR_EN_0_MASK                                                     (0x00000001U)
#define ADC_DMAENABLE_CLR_EN_0_ENABLED                                                   (1U)
#define ADC_DMAENABLE_CLR_EN_0_DISABLED                                                  (0U)
#define ADC_DMAENABLE_CLR_EN_0_DISABLE                                                   (1U)
#define ADC_DMAENABLE_CLR_EN_0_NO_ACTION                                                 (0U)

#define ADC_CTRL_STEP_ID_TAG_SHIFT                                                      (1U)
#define ADC_CTRL_STEP_ID_TAG_MASK                                                       (0x00000002U)
#define ADC_CTRL_STEP_ID_TAG_WRZERO                                                      (0U)
#define ADC_CTRL_STEP_ID_TAG_CHANNELID                                                   (1U)

#define ADC_CTRL_EN_SHIFT                                                               (0U)
#define ADC_CTRL_EN_MASK                                                                (0x00000001U)
#define ADC_CTRL_EN_DISABLE                                                              (0U)
#define ADC_CTRL_EN_ENABLE                                                               (1U)

#define ADC_CTRL_POWER_DOWN_SHIFT                                                       (4U)
#define ADC_CTRL_POWER_DOWN_MASK                                                        (0x00000010U)
#define ADC_CTRL_POWER_DOWN_AFEPOWERUP                                                   (0U)
#define ADC_CTRL_POWER_DOWN_AFEPOWERDOWN                                                 (1U)

#define ADC_CTRL_HW_PREEMPT_SHIFT                                                       (9U)
#define ADC_CTRL_HW_PREEMPT_MASK                                                        (0x00000200U)
#define ADC_CTRL_HW_PREEMPT_NOPREEMPT                                                    (0U)
#define ADC_CTRL_HW_PREEMPT                                                              (1U)

#define ADC_CTRL_AFE_PEN_SHIFT                                                          (5U)
#define ADC_CTRL_AFE_PEN_MASK                                                           (0x00000060U)

#define ADC_CTRL_RESERVED0_SHIFT                                                         (10U)
#define ADC_CTRL_RESERVED0_MASK                                                          (0xfffffc00U)

#define ADC_CTRL_TOUCH_SCREEN_EN_SHIFT                                                  (7U)
#define ADC_CTRL_TOUCH_SCREEN_EN_MASK                                                   (0x00000080U)
#define ADC_CTRL_TOUCH_SCREEN_EN_DISABLE                                                 (0U)
#define ADC_CTRL_TOUCH_SCREEN_EN_ENABLE                                                  (1U)

#define ADC_CTRL_HW_EVT_MAPPING_SHIFT                                                   (8U)
#define ADC_CTRL_HW_EVT_MAPPING_MASK                                                    (0x00000100U)
#define ADC_CTRL_HW_EVT_MAPPING_PENTOUCHIRQ                                              (0U)
#define ADC_CTRL_HW_EVT_MAPPING_HWEVTINPUT                                               (1U)

#define ADC_CTRL_BIAS_SELECT_SHIFT                                                      (3U)
#define ADC_CTRL_BIAS_SELECT_MASK                                                       (0x00000008U)
#define ADC_CTRL_BIAS_SELECT_INTERNAL                                                    (0U)
#define ADC_CTRL_BIAS_SELECT_EXTERNAL                                                    (1U)

#define ADC_CTRL_STEPCONFIG_WRITEPROTECT_N_SHIFT                                        (2U)
#define ADC_CTRL_STEPCONFIG_WRITEPROTECT_N_MASK                                         (0x00000004U)
#define ADC_CTRL_STEPCONFIG_WRITEPROTECT_N_PROTECTED                                     (0U)
#define ADC_CTRL_STEPCONFIG_WRITEPROTECT_N_NOTPROTECTED                                  (1U)

#define ADC_CTRL_HW_MID_EN_SHIFT                                                       (10U)
#define ADC_CTRL_HW_MID_EN_MASK                                                        (0x00000400U)
#define ADC_CTRL_HW_MID_EN_DISABLE                                                      (0U)
#define ADC_CTRL_HW_MID_EN_ENABLE                                                       (1U)

#define ADC_CTRL_HW_MID_SEL_SHIFT                                                      (11U)
#define ADC_CTRL_HW_MID_SEL_MASK                                                       (0x00000800U)
#define ADC_CTRL_HW_MID_SEL_DISABLE                                                     (0U)
#define ADC_CTRL_HW_MID_SEL_ENABLE                                                      (1U)

#define ADC_ADCSTAT_MEM_INIT_DONE_SHIFT                                                (6U)
#define ADC_ADCSTAT_MEM_INIT_DONE_MASK                                                 (0x00000040U)
#define ADC_ADCSTAT_MEM_INIT_DONE_RESETVAL                                              (0U)
#define ADC_ADCSTAT_MEM_INIT_DONE                                                       (1U)

#define ADC_ADCSTAT_RESERVED0_SHIFT                                                      (9U)
#define ADC_ADCSTAT_RESERVED0_MASK                                                       (0xfffffe00U)

#define ADC_ADCSTAT_AFE_BUSY_SHIFT                                                      (8U)
#define ADC_ADCSTAT_AFE_BUSY_MASK                                                       (0x00000100U)
#define ADC_ADCSTAT_AFE_BUSY_IDLE                                                        (0U)
#define ADC_ADCSTAT_AFE_BUSY                                                             (1U)

#define ADC_ADCSTAT_FSM_BUSY_SHIFT                                                      (5U)
#define ADC_ADCSTAT_FSM_BUSY_MASK                                                       (0x00000020U)
#define ADC_ADCSTAT_FSM_BUSY_IDLE                                                        (0U)
#define ADC_ADCSTAT_FSM_BUSY                                                             (1U)

#define ADC_ADCSTAT_STEP_ID_SHIFT                                                       (0U)
#define ADC_ADCSTAT_STEP_ID_MASK                                                        (0x0000001fU)
#define ADC_ADCSTAT_STEP_ID_STEP1                                                        (0U)
#define ADC_ADCSTAT_STEP_ID_STEP2                                                        (1U)
#define ADC_ADCSTAT_STEP_ID_STEP3                                                        (2U)
#define ADC_ADCSTAT_STEP_ID_STEP4                                                        (3U)
#define ADC_ADCSTAT_STEP_ID_STEP5                                                        (4U)
#define ADC_ADCSTAT_STEP_ID_STEP6                                                        (5U)
#define ADC_ADCSTAT_STEP_ID_STEP7                                                        (6U)
#define ADC_ADCSTAT_STEP_ID_STEP8                                                        (7U)
#define ADC_ADCSTAT_STEP_ID_STEP9                                                        (8U)
#define ADC_ADCSTAT_STEP_ID_STEP10                                                       (9U)
#define ADC_ADCSTAT_STEP_ID_STEP11                                                       (10U)
#define ADC_ADCSTAT_STEP_ID_STEP12                                                       (11U)
#define ADC_ADCSTAT_STEP_ID_STEP13                                                       (12U)
#define ADC_ADCSTAT_STEP_ID_STEP14                                                       (13U)
#define ADC_ADCSTAT_STEP_ID_STEP15                                                       (14U)
#define ADC_ADCSTAT_STEP_ID_STEP16                                                       (15U)
#define ADC_ADCSTAT_STEP_ID_IDLE                                                         (16U)
#define ADC_ADCSTAT_STEP_ID_CHARGE                                                       (17U)

#define ADC_ADCSTAT_PEN_IRQ1_SHIFT                                                      (7U)
#define ADC_ADCSTAT_PEN_IRQ1_MASK                                                       (0x00000080U)

#define ADC_ADCRANGE_RESERVED1_SHIFT                                                     (28U)
#define ADC_ADCRANGE_RESERVED1_MASK                                                      (0xf0000000U)

#define ADC_ADCRANGE_HIGH_RANGE_DATA_SHIFT                                              (16U)
#define ADC_ADCRANGE_HIGH_RANGE_DATA_MASK                                               (0x0fff0000U)

#define ADC_ADCRANGE_LOW_RANGE_DATA_SHIFT                                               (0U)
#define ADC_ADCRANGE_LOW_RANGE_DATA_MASK                                                (0x00000fffU)

#define ADC_ADCRANGE_RESERVED0_SHIFT                                                     (12U)
#define ADC_ADCRANGE_RESERVED0_MASK                                                      (0x0000f000U)

#define ADC_MISC_AFE_SPARE_INPUT_SHIFT                                                  (0U)
#define ADC_MISC_AFE_SPARE_INPUT_MASK                                                   (0x0000000fU)

#define ADC_MISC_RESERVED0_SHIFT                                                         (8U)
#define ADC_MISC_RESERVED0_MASK                                                          (0xffffff00U)

#define ADC_MISC_AFE_SPARE_OUTPUT_SHIFT                                                 (8U)
#define ADC_MISC_AFE_SPARE_OUTPUT_MASK                                                  (0x00000f00U)

#define ADC_STEPENABLE_STEP_SHIFT                                                      (1U)
#define ADC_STEPENABLE_STEP_MASK                                                       (0x0001FFFEU)

#define ADC_STEPENABLE_STEP3_SHIFT                                                      (3U)
#define ADC_STEPENABLE_STEP3_MASK                                                       (0x00000008U)

#define ADC_STEPENABLE_STEP8_SHIFT                                                      (8U)
#define ADC_STEPENABLE_STEP8_MASK                                                       (0x00000100U)

#define ADC_STEPENABLE_STEP2_SHIFT                                                      (2U)
#define ADC_STEPENABLE_STEP2_MASK                                                       (0x00000004U)

#define ADC_STEPENABLE_STEP16_SHIFT                                                     (16U)
#define ADC_STEPENABLE_STEP16_MASK                                                      (0x00010000U)

#define ADC_STEPENABLE_STEP15_SHIFT                                                     (15U)
#define ADC_STEPENABLE_STEP15_MASK                                                      (0x00008000U)

#define ADC_STEPENABLE_STEP12_SHIFT                                                     (12U)
#define ADC_STEPENABLE_STEP12_MASK                                                      (0x00001000U)

#define ADC_STEPENABLE_STEP13_SHIFT                                                     (13U)
#define ADC_STEPENABLE_STEP13_MASK                                                      (0x00002000U)

#define ADC_STEPENABLE_RESERVED0_SHIFT                                                   (17U)
#define ADC_STEPENABLE_RESERVED0_MASK                                                    (0xfffe0000U)

#define ADC_STEPENABLE_STEP14_SHIFT                                                     (14U)
#define ADC_STEPENABLE_STEP14_MASK                                                      (0x00004000U)

#define ADC_STEPENABLE_STEP7_SHIFT                                                      (7U)
#define ADC_STEPENABLE_STEP7_MASK                                                       (0x00000080U)

#define ADC_STEPENABLE_STEP9_SHIFT                                                      (9U)
#define ADC_STEPENABLE_STEP9_MASK                                                       (0x00000200U)

#define ADC_STEPENABLE_STEP10_SHIFT                                                     (10U)
#define ADC_STEPENABLE_STEP10_MASK                                                      (0x00000400U)

#define ADC_STEPENABLE_STEP4_SHIFT                                                      (4U)
#define ADC_STEPENABLE_STEP4_MASK                                                       (0x00000010U)

#define ADC_STEPENABLE_STEP1_SHIFT                                                      (1U)
#define ADC_STEPENABLE_STEP1_MASK                                                       (0x00000002U)

#define ADC_STEPENABLE_STEP5_SHIFT                                                      (5U)
#define ADC_STEPENABLE_STEP5_MASK                                                       (0x00000020U)

#define ADC_STEPENABLE_STEP11_SHIFT                                                     (11U)
#define ADC_STEPENABLE_STEP11_MASK                                                      (0x00000800U)

#define ADC_STEPENABLE_STEP6_SHIFT                                                      (6U)
#define ADC_STEPENABLE_STEP6_MASK                                                       (0x00000040U)

#define ADC_STEPENABLE_TS_CHARGE_SHIFT                                                  (0U)
#define ADC_STEPENABLE_TS_CHARGE_MASK                                                   (0x00000001U)

#define ADC_IDLECONFIG_XPPSW_SWC_SHIFT                                                  (5U)
#define ADC_IDLECONFIG_XPPSW_SWC_MASK                                                   (0x00000020U)

#define ADC_IDLECONFIG_SEL_RFP_SWC_SHIFT                                                (12U)
#define ADC_IDLECONFIG_SEL_RFP_SWC_MASK                                                 (0x00007000U)
#define ADC_IDLECONFIG_SEL_RFP_SWC_VDDA                                                  (0U)
#define ADC_IDLECONFIG_SEL_RFP_SWC_XPUL                                                  (1U)
#define ADC_IDLECONFIG_SEL_RFP_SWC_YPLL                                                  (2U)
#define ADC_IDLECONFIG_SEL_RFP_SWC_VREFP                                                 (3U)

#define ADC_IDLECONFIG_SEL_INM_SWM_SHIFT                                                (15U)
#define ADC_IDLECONFIG_SEL_INM_SWM_MASK                                                 (0x00078000U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_1                                             (0U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_2                                             (1U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_3                                             (2U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_4                                             (3U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_5                                             (4U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_6                                             (5U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_7                                             (6U)
#define ADC_IDLECONFIG_SEL_INM_SWM_CHANNEL_8                                             (7U)
#define ADC_IDLECONFIG_SEL_INM_SWM_VREFN                                                 (8U)

#define ADC_IDLECONFIG_RESERVED0_SHIFT                                                   (0U)
#define ADC_IDLECONFIG_RESERVED0_MASK                                                    (0x0000001fU)

#define ADC_IDLECONFIG_YPPSW_SWC_SHIFT                                                  (7U)
#define ADC_IDLECONFIG_YPPSW_SWC_MASK                                                   (0x00000080U)

#define ADC_IDLECONFIG_YPNSW_SWC_SHIFT                                                  (10U)
#define ADC_IDLECONFIG_YPNSW_SWC_MASK                                                   (0x00000400U)

#define ADC_IDLECONFIG_RESERVED1_SHIFT                                                   (26U)
#define ADC_IDLECONFIG_RESERVED1_MASK                                                    (0xfc000000U)

#define ADC_IDLECONFIG_XNNSW_SWC_SHIFT                                                  (6U)
#define ADC_IDLECONFIG_XNNSW_SWC_MASK                                                   (0x00000040U)

#define ADC_IDLECONFIG_XNPSW_SWC_SHIFT                                                  (9U)
#define ADC_IDLECONFIG_XNPSW_SWC_MASK                                                   (0x00000200U)

#define ADC_IDLECONFIG_YNNSW_SWC_SHIFT                                                  (8U)
#define ADC_IDLECONFIG_YNNSW_SWC_MASK                                                   (0x00000100U)

#define ADC_IDLECONFIG_SEL_RFM_SWC_SHIFT                                                (23U)
#define ADC_IDLECONFIG_SEL_RFM_SWC_MASK                                                 (0x01800000U)
#define ADC_IDLECONFIG_SEL_RFM_SWC_VSSA                                                  (0U)
#define ADC_IDLECONFIG_SEL_RFM_SWC_XNUR                                                  (1U)
#define ADC_IDLECONFIG_SEL_RFM_SWC_YNLR                                                  (2U)
#define ADC_IDLECONFIG_SEL_RFM_SWC_VREFN                                                 (3U)

#define ADC_IDLECONFIG_WPNSW_SWC_SHIFT                                                  (11U)
#define ADC_IDLECONFIG_WPNSW_SWC_MASK                                                   (0x00000800U)

#define ADC_IDLECONFIG_SEL_INP_SWC_SHIFT                                                (19U)
#define ADC_IDLECONFIG_SEL_INP_SWC_MASK                                                 (0x00780000U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_1                                             (0U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_2                                             (1U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_3                                             (2U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_4                                             (3U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_5                                             (4U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_6                                             (5U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_7                                             (6U)
#define ADC_IDLECONFIG_SEL_INP_SWC_CHANNEL_8                                             (7U)
#define ADC_IDLECONFIG_SEL_INP_SWC_VREFN                                                 (8U)

#define ADC_IDLECONFIG_DIFF_CNTRL_SHIFT                                                 (25U)
#define ADC_IDLECONFIG_DIFF_CNTRL_MASK                                                  (0x02000000U)
#define ADC_IDLECONFIG_DIFF_CNTRL_SINGLE                                                 (0U)
#define ADC_IDLECONFIG_DIFF_CNTRL_DIFFERENTIAL                                           (1U)

#define ADC_TS_CHARGE_STEPCONFIG_DIFF_CNTRL_SHIFT                                       (25U)
#define ADC_TS_CHARGE_STEPCONFIG_DIFF_CNTRL_MASK                                        (0x02000000U)
#define ADC_TS_CHARGE_STEPCONFIG_DIFF_CNTRL_SINGLE                                       (0U)
#define ADC_TS_CHARGE_STEPCONFIG_DIFF_CNTRL_DIFFERENTIAL                                 (1U)

#define ADC_TS_CHARGE_STEPCONFIG_XNPSW_SWC_SHIFT                                        (9U)
#define ADC_TS_CHARGE_STEPCONFIG_XNPSW_SWC_MASK                                         (0x00000200U)

#define ADC_TS_CHARGE_STEPCONFIG_YPNSW_SWC_SHIFT                                        (10U)
#define ADC_TS_CHARGE_STEPCONFIG_YPNSW_SWC_MASK                                         (0x00000400U)

#define ADC_TS_CHARGE_STEPCONFIG_XPPSW_SWC_SHIFT                                        (5U)
#define ADC_TS_CHARGE_STEPCONFIG_XPPSW_SWC_MASK                                         (0x00000020U)

#define ADC_TS_CHARGE_STEPCONFIG_RESERVED0_SHIFT                                         (0U)
#define ADC_TS_CHARGE_STEPCONFIG_RESERVED0_MASK                                          (0x0000001fU)

#define ADC_TS_CHARGE_STEPCONFIG_RESERVED1_SHIFT                                         (26U)
#define ADC_TS_CHARGE_STEPCONFIG_RESERVED1_MASK                                          (0xfc000000U)

#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_SHIFT                                      (15U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_MASK                                       (0x00078000U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_1                                   (0U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_2                                   (1U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_3                                   (2U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_4                                   (3U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_5                                   (4U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_6                                   (5U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_7                                   (6U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_CHANNEL_8                                   (7U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INM_SWM_VREFN                                       (8U)

#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_SHIFT                                      (12U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_MASK                                       (0x00007000U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_VDDA                                        (0U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_XPUL                                        (1U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_YPLL                                        (2U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_VREFP                                       (3U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFP_SWC_INTREF                                      (4U)

#define ADC_TS_CHARGE_STEPCONFIG_WPNSW_SWC_SHIFT                                        (11U)
#define ADC_TS_CHARGE_STEPCONFIG_WPNSW_SWC_MASK                                         (0x00000800U)

#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_SHIFT                                      (19U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_MASK                                       (0x00780000U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_1                                   (0U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_2                                   (1U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_3                                   (2U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_4                                   (3U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_5                                   (4U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_6                                   (5U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_7                                   (6U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_CHANNEL_8                                   (7U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_INP_SWC_VREFN                                       (8U)

#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_SHIFT                                      (23U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_MASK                                       (0x01800000U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_VSSA                                        (0U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_XNUR                                        (1U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_YNLR                                        (2U)
#define ADC_TS_CHARGE_STEPCONFIG_SEL_RFM_SWC_VREFN                                       (3U)

#define ADC_TS_CHARGE_STEPCONFIG_YPPSW__SWC_SHIFT                                       (7U)
#define ADC_TS_CHARGE_STEPCONFIG_YPPSW__SWC_MASK                                        (0x00000080U)

#define ADC_TS_CHARGE_STEPCONFIG_XNNSW__SWC_SHIFT                                       (6U)
#define ADC_TS_CHARGE_STEPCONFIG_XNNSW__SWC_MASK                                        (0x00000040U)

#define ADC_TS_CHARGE_STEPCONFIG_YNNSW_SWC_SHIFT                                        (8U)
#define ADC_TS_CHARGE_STEPCONFIG_YNNSW_SWC_MASK                                         (0x00000100U)

#define ADC_TS_CHARGE_DELAY_OPENDELAY_SHIFT                                             (0U)
#define ADC_TS_CHARGE_DELAY_OPENDELAY_MASK                                              (0x0003ffffU)

#define ADC_TS_CHARGE_DELAY_RESERVED0_SHIFT                                              (18U)
#define ADC_TS_CHARGE_DELAY_RESERVED0_MASK                                               (0xfffc0000U)

#define ADC_IRQ_EOI_LINE_NUMBER_SHIFT                                                   (0U)
#define ADC_IRQ_EOI_LINE_NUMBER_MASK                                                    (0x00000001U)
#define ADC_IRQ_EOI_LINE_NUMBER_EOI                                                     (0U)

#define ADC_IRQ_EOI_RESERVED0_SHIFT                                                      (1U)
#define ADC_IRQ_EOI_RESERVED0_MASK                                                       (0x7ffffffeU)

#define ADC_STEPCONFIG_YNNSW_SWC_SHIFT                                                  (8U)
#define ADC_STEPCONFIG_YNNSW_SWC_MASK                                                   (0x00000100U)

#define ADC_STEPCONFIG_DIFF_CNTRL_SHIFT                                                 (25U)
#define ADC_STEPCONFIG_DIFF_CNTRL_MASK                                                  (0x02000000U)
#define ADC_STEPCONFIG_DIFF_CNTRL_SINGLE                                                (0U)
#define ADC_STEPCONFIG_DIFF_CNTRL_DIFFERENTIAL                                          (1U)

#define ADC_STEPCONFIG_WPNSW_SWC_SHIFT                                                  (11U)
#define ADC_STEPCONFIG_WPNSW_SWC_MASK                                                   (0x00000800U)

#define ADC_STEPCONFIG_MODE_SHIFT                                                       (0U)
#define ADC_STEPCONFIG_MODE_MASK                                                        (0x00000003U)
#define ADC_STEPCONFIG_MODE_SW_EN_ONESHOT                                                (0U)
#define ADC_STEPCONFIG_MODE_SW_EN_CONTINUOUS                                             (1U)
#define ADC_STEPCONFIG_MODE_HW_SYNC_ONESHOT                                              (2U)
#define ADC_STEPCONFIG_MODE_HW_SYNC_CONTINUOUS                                           (3U)

#define ADC_STEPCONFIG_AVERAGING_SHIFT                                                  (2U)
#define ADC_STEPCONFIG_AVERAGING_MASK                                                   (0x0000001cU)
#define ADC_STEPCONFIG_AVERAGING_NOAVG                                                  (0U)
#define ADC_STEPCONFIG_AVERAGING_2_SAMPLESAVG                                           (1U)
#define ADC_STEPCONFIG_AVERAGING_4_SAMPLESAVG                                           (2U)
#define ADC_STEPCONFIG_AVERAGING_8_SAMPLESAVG                                           (3U)
#define ADC_STEPCONFIG_AVERAGING_16_SAMPLESAV                                           (4U)

#define ADC_STEPCONFIG_XPPSW_SWC_SHIFT                                                  (5U)
#define ADC_STEPCONFIG_XPPSW_SWC_MASK                                                   (0x00000020U)

#define ADC_STEPCONFIG_YPPSW_SWC_SHIFT                                                  (7U)
#define ADC_STEPCONFIG_YPPSW_SWC_MASK                                                   (0x00000080U)

#define ADC_STEPCONFIG_XNNSW_SWC_SHIFT                                                  (6U)
#define ADC_STEPCONFIG_XNNSW_SWC_MASK                                                   (0x00000040U)

#define ADC_STEPCONFIG_FIFO_SELECT_SHIFT                                                (26U)
#define ADC_STEPCONFIG_FIFO_SELECT_MASK                                                 (0x04000000U)
#define ADC_STEPCONFIG_FIFO_SELECT_0                                                     (0U)
#define ADC_STEPCONFIG_FIFO_SELECT_1                                                     (1U)

#define ADC_STEPCONFIG_RESERVED0_SHIFT                                                   (28U)
#define ADC_STEPCONFIG_RESERVED0_MASK                                                    (0xf0000000U)

#define ADC_STEPCONFIG_RANGE_CHECK_SHIFT                                                (27U)
#define ADC_STEPCONFIG_RANGE_CHECK_MASK                                                 (0x08000000U)
#define ADC_STEPCONFIG_RANGE_CHECK_DISABLE                                               (0U)
#define ADC_STEPCONFIG_RANGE_CHECK_ENABLE                                                (1U)

#define ADC_STEPCONFIG_YPNSW_SWC_SHIFT                                                  (10U)
#define ADC_STEPCONFIG_YPNSW_SWC_MASK                                                   (0x00000400U)

#define ADC_STEPCONFIG_XNPSW_SWC_SHIFT                                                  (9U)
#define ADC_STEPCONFIG_XNPSW_SWC_MASK                                                   (0x00000200U)

#define ADC_STEPCONFIG_SEL_RFP_SWC_SHIFT                                                (12U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_MASK                                                 (0x00007000U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_VDDA                                                  (0U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_XPUL                                                  (1U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_YPLL                                                  (2U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_VREFP                                                 (3U)
#define ADC_STEPCONFIG_SEL_RFP_SWC_INTREF                                                (4U)

#define ADC_STEPCONFIG_SEL_INM_SWC_SHIFT                                                (15U)
#define ADC_STEPCONFIG_SEL_INM_SWC_MASK                                                 (0x00078000U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_1                                             (0U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_2                                             (1U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_3                                             (2U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_4                                             (3U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_5                                             (4U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_6                                             (5U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_7                                             (6U)
#define ADC_STEPCONFIG_SEL_INM_SWC_CHANNEL_8                                             (7U)
#define ADC_STEPCONFIG_SEL_INM_SWC_VREFN                                                 (8U)

#define ADC_STEPCONFIG_SEL_INP_SWC_SHIFT                                                (19U)
#define ADC_STEPCONFIG_SEL_INP_SWC_MASK                                                 (0x00780000U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_1                                             (0U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_2                                             (1U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_3                                             (2U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_4                                             (3U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_5                                             (4U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_6                                             (5U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_7                                             (6U)
#define ADC_STEPCONFIG_SEL_INP_SWC_CHANNEL_8                                             (7U)
#define ADC_STEPCONFIG_SEL_INP_SWC_VREFN                                                 (8U)

#define ADC_STEPCONFIG_SEL_RFM_SWC_SHIFT                                                (23U)
#define ADC_STEPCONFIG_SEL_RFM_SWC_MASK                                                 (0x01800000U)
#define ADC_STEPCONFIG_SEL_RFM_SWC_VSSA                                                  (0U)
#define ADC_STEPCONFIG_SEL_RFM_SWC_XNUR                                                  (1U)
#define ADC_STEPCONFIG_SEL_RFM_SWC_YNLR                                                  (2U)
#define ADC_STEPCONFIG_SEL_RFM_SWC_VREFN                                                 (3U)

#define ADC_STEPDELAY_RESERVED0_SHIFT                                                    (18U)
#define ADC_STEPDELAY_RESERVED0_MASK                                                     (0x00fc0000U)

#define ADC_STEPDELAY_OPENDELAY_SHIFT                                                   (0U)
#define ADC_STEPDELAY_OPENDELAY_MASK                                                    (0x0003ffffU)

#define ADC_STEPDELAY_SAMPLEDELAY_SHIFT                                                 (24U)
#define ADC_STEPDELAY_SAMPLEDELAY_MASK                                                  (0xff000000U)

#define ADC_FIFOCOUNT_WORDS_IN_FIFO_SHIFT                                               (0U)
#define ADC_FIFOCOUNT_WORDS_IN_FIFO_MASK                                                (0x000001ffU)

#define ADC_FIFOCOUNT_RESERVED0_SHIFT                                                   (7U)
#define ADC_FIFOCOUNT_RESERVED0_MASK                                                    (0xffffff80U)

#define ADC_FIFOTHRESHOLD_RESERVED0_SHIFT                                               (6U)
#define ADC_FIFOTHRESHOLD_RESERVED0_MASK                                                (0xffffffc0U)

#define ADC_FIFOTHRESHOLD_FIFO_THR_LEVEL_SHIFT                                          (0U)
#define ADC_FIFOTHRESHOLD_FIFO_THR_LEVEL_MASK                                           (0x000000ffU)
#define ADC_FIFOTHRESHOLD_FIFO_THR_LEVEL_MAX                                            (256U)

#define ADC_DMAREQ_DMA_REQUEST_LEVEL_SHIFT                                              (0U)
#define ADC_DMAREQ_DMA_REQUEST_LEVEL_MASK                                               (0x0000003fU)

#define ADC_DMAREQ_RESERVED0_SHIFT                                                      (6U)
#define ADC_DMAREQ_RESERVED0_MASK                                                       (0xffffffc0U)

#define ADC_FIFODATA_RESERVED1_SHIFT                                                    (20U)
#define ADC_FIFODATA_RESERVED1_MASK                                                     (0xfff00000U)

#define ADC_FIFODATA_ADCCHNLID_SHIFT                                                    (16U)
#define ADC_FIFODATA_ADCCHNLID_MASK                                                     (0x000f0000U)

#define ADC_FIFODATA_ADCDATA_SHIFT                                                      (0U)
#define ADC_FIFODATA_ADCDATA_MASK                                                       (0x00000fffU)

#define ADC_FIFODATA_RESERVED0_SHIFT                                                    (12U)
#define ADC_FIFODATA_RESERVED0_MASK                                                     (0x0000f000U)

#define ADC_SYSCONFIG_IDLEMODE_SHIFT                                                    (2U)
#define ADC_SYSCONFIG_IDLEMODE_MASK                                                     (0x0000000cU)
#define ADC_SYSCONFIG_IDLEMODE_FORCE                                                     (0U)
#define ADC_SYSCONFIG_IDLEMODE_NO_IDLE                                                   (1U)
#define ADC_SYSCONFIG_IDLEMODE_SMART_IDLE                                                (2U)
#define ADC_SYSCONFIG_IDLEMODE_SMART_IDLE_WAKEUP                                         (3U)

#define ADC_SYSCONFIG_RESERVED1_SHIFT                                                    (4U)
#define ADC_SYSCONFIG_RESERVED1_MASK                                                     (0xfffffff0U)

#define ADC_SYSCONFIG_RESERVED0_SHIFT                                                    (0U)
#define ADC_SYSCONFIG_RESERVED0_MASK                                                     (0x00000003U)

#ifdef __cplusplus
}
#endif
#endif  /* HW_ADC_H_ */
