/*
 *  Copyright (C)2018-2021 Texas Instruments Incorporated
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

/**
 *  \file       hw_i2c.h
 *
 *  \brief      register-level header file for I2C
 *
**/

#ifndef CSLR_I2C_H_
#define CSLR_I2C_H_

#include <drivers/hw_include/cslr.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
*                        Register Overlay Structure 
**************************************************************************/
typedef struct {
    volatile Uint16 REVNB_LO;
    volatile Uint8  RSVD0[2];
    volatile Uint16 REVNB_HI;
    volatile Uint8  RSVD1[10];
    volatile Uint16 SYSC;
    volatile Uint8  RSVD2[14];
    volatile Uint16 EOI;
    volatile Uint8  RSVD3[2];
    volatile Uint16 IRQSTATUS_RAW;
    volatile Uint8  RSVD4[2];
    volatile Uint16 IRQSTATUS;
    volatile Uint8  RSVD5[2];
    volatile Uint16 IRQENABLE_SET;
    volatile Uint8  RSVD6[2];
    volatile Uint16 IRQENABLE_CLR;
    volatile Uint8  RSVD7[2];
    volatile Uint16 WE;
    volatile Uint8  RSVD8[2];
    volatile Uint16 DMARXENABLE_SET;
    volatile Uint8  RSVD9[2];
    volatile Uint16 DMATXENABLE_SET;
    volatile Uint8  RSVD10[2];
    volatile Uint16 DMARXENABLE_CLR;
    volatile Uint8  RSVD11[2];
    volatile Uint16 DMATXENABLE_CLR;
    volatile Uint8  RSVD12[2];
    volatile Uint16 DMARXWAKE_EN;
    volatile Uint8  RSVD13[2];
    volatile Uint16 DMATXWAKE_EN;
    volatile Uint8  RSVD14[54];
    volatile Uint16 IE;
    volatile Uint8  RSVD15[2];
    volatile Uint16 STAT;
    volatile Uint8  RSVD16[6];
    volatile Uint16 SYSS;
    volatile Uint8  RSVD17[2];
    volatile Uint16 BUF;
    volatile Uint8  RSVD18[2];
    volatile Uint16 CNT;
    volatile Uint8  RSVD19[2];
    volatile Uint16 DATA;
    volatile Uint8  RSVD20[6];
    volatile Uint16 CON;
    volatile Uint8  RSVD21[2];
    volatile Uint16 OA;
    volatile Uint8  RSVD22[2];
    volatile Uint16 SA;
    volatile Uint8  RSVD23[2];
    volatile Uint16 PSC;
    volatile Uint8  RSVD24[2];
    volatile Uint16 SCLL;
    volatile Uint8  RSVD25[2];
    volatile Uint16 SCLH;
    volatile Uint8  RSVD26[2];
    volatile Uint16 SYSTEST;
    volatile Uint8  RSVD27[2];
    volatile Uint16 BUFSTAT;
    volatile Uint8  RSVD28[2];
    volatile Uint16 OA1;
    volatile Uint8  RSVD29[2];
    volatile Uint16 OA2;
    volatile Uint8  RSVD30[2];
    volatile Uint16 OA3;
    volatile Uint8  RSVD31[2];
    volatile Uint16 ACTOA;
    volatile Uint8  RSVD32[2];
    volatile Uint16 SBLOCK;
} CSL_I2cRegs;

/**************************************************************************
*                        Register Definitions
***************************************************************************/

/* Revision Number register (Low) */
#define CSL_I2C_REVNB_LO                                        (0x0U)

/* Revision Number register (High) */
#define CSL_I2C_REVNB_HI                                        (0x4U)

/* System Configuration register */
#define CSL_I2C_SYSC                                            (0x10U)

/* End Of Interrupt number specification */
#define CSL_I2C_EOI                                             (0x20U)

/* Per-event raw interrupt status vector */
#define CSL_I2C_IRQSTATUS_RAW                                   (0x24U)

/* Per-event enabled interrupt status vector */
#define CSL_I2C_IRQSTATUS                                       (0x28U)

/* Per-event interrupt enable bit vector. */
#define CSL_I2C_IRQENABLE_SET                                   (0x2CU)

/* Per-event interrupt clear bit vector. */
#define CSL_I2C_IRQENABLE_CLR                                   (0x30U)

/* I2C wakeup enable vector (legacy). */
#define CSL_I2C_WE                                              (0x34U)

/* Per-event DMA RX enable set. */
#define CSL_I2C_DMARXENABLE_SET                                 (0x38U)

/* Per-event DMA TX enable set. */
#define CSL_I2C_DMATXENABLE_SET                                 (0x3CU)

/* Per-event DMA RX enable clear. */
#define CSL_I2C_DMARXENABLE_CLR                                 (0x40U)

/* Per-event DMA TX enable clear. */
#define CSL_I2C_DMATXENABLE_CLR                                 (0x44U)

/* Per-event DMA RX wakeup enable. */
#define CSL_I2C_DMARXWAKE_EN                                    (0x48U)

/* Per-event DMA TX wakeup enable. */
#define CSL_I2C_DMATXWAKE_EN                                    (0x4CU)

/* I2C interrupt enable vector (legacy). */
#define CSL_I2C_IE                                              (0x84U)

/* I2C interrupt status vector (legacy). */
#define CSL_I2C_STAT                                            (0x88U)

/* System Status register */
#define CSL_I2C_SYSS                                            (0x90U)

/* Buffer Configuration register */
#define CSL_I2C_BUF                                             (0x94U)

/* Data counter register */
#define CSL_I2C_CNT                                             (0x98U)

/* Data access register */
#define CSL_I2C_DATA                                            (0x9CU)

/* I2C configuration register. */
#define CSL_I2C_CON                                             (0xA4U)

/* Own address register */
#define CSL_I2C_OA                                              (0xA8U)

/* Slave address register */
#define CSL_I2C_SA                                              (0xACU)

/* I2C Clock Prescaler Register */
#define CSL_I2C_PSC                                             (0xB0U)

/* I2C SCL Low Time Register. */
#define CSL_I2C_SCLL                                            (0xB4U)

/* I2C SCL High Time Register. */
#define CSL_I2C_SCLH                                            (0xB8U)

/* I2C System Test Register. */
#define CSL_I2C_SYSTEST                                         (0xBCU)

/* I2C Buffer Status Register. */
#define CSL_I2C_BUFSTAT                                         (0xC0U)

/* I2C Own Address 1 Register */
#define CSL_I2C_OA1                                             (0xC4U)

/* I2C Own Address 2 Register */
#define CSL_I2C_OA2                                             (0xC8U)

/* I2C Own Address 3 Register */
#define CSL_I2C_OA3                                             (0xCCU)

/* I2C Active Own Address Register. */
#define CSL_I2C_ACTOA                                           (0xD0U)

/* I2C Clock Blocking Enable Register. */
#define CSL_I2C_SBLOCK                                          (0xD4U)


#define I2C_REVNB_LO                                             (0x0U)
#define I2C_REVNB_HI                                             (0x4U)
#define I2C_SYSC                                                (0x10U)
#define I2C_IRQSTATUS_RAW                                       (0x24U)
#define I2C_IRQSTATUS                                           (0x28U)
#define I2C_IRQENABLE_SET                                       (0x2cU)
#define I2C_IRQENABLE_CLR                                       (0x30U)
#define I2C_WE                                                  (0x34U)
#define I2C_DMARXENABLE_SET                                     (0x38U)
#define I2C_DMATXENABLE_SET                                     (0x3cU)
#define I2C_DMARXENABLE_CLR                                     (0x40U)
#define I2C_DMATXENABLE_CLR                                     (0x44U)
#define I2C_DMARXWAKE_EN                                        (0x48U)
#define I2C_DMATXWAKE_EN                                        (0x4cU)
#define I2C_SYSS                                                (0x90U)
#define I2C_BUF                                                 (0x94U)
#define I2C_CNT                                                 (0x98U)
#define I2C_DATA                                                (0x9cU)
#define I2C_CON                                                 (0xa4U)
#define I2C_OA                                                  (0xa8U)
#define I2C_SA                                                  (0xacU)
#define I2C_PSC                                                 (0xb0U)
#define I2C_SCLL                                                (0xb4U)
#define I2C_SCLH                                                (0xb8U)
#define I2C_SYSTEST                                             (0xbcU)
#define I2C_BUFSTAT                                             (0xc0U)
#define I2C_OA1                                                 (0xc4U)
#define I2C_OA2                                                 (0xc8U)
#define I2C_OA3                                                 (0xccU)
#define I2C_ACTOA                                               (0xd0U)
#define I2C_SBLOCK                                              (0xd4U)

/**************************************************************************
*                          Field Definition Macros
***************************************************************************/

/* REVNB_LO */

#define CSL_I2C_REVNB_LO_MINOR_MASK                             (0x0000003FU)
#define CSL_I2C_REVNB_LO_MINOR_SHIFT                            (0U)
#define CSL_I2C_REVNB_LO_MINOR_RESETVAL                         (0x00000003U)
#define CSL_I2C_REVNB_LO_MINOR_MAX                              (0x0000003fU)

#define CSL_I2C_REVNB_LO_CUSTOM_MASK                            (0x000000C0U)
#define CSL_I2C_REVNB_LO_CUSTOM_SHIFT                           (6U)
#define CSL_I2C_REVNB_LO_CUSTOM_RESETVAL                        (0x00000000U)
#define CSL_I2C_REVNB_LO_CUSTOM_MAX                             (0x00000003U)

#define CSL_I2C_REVNB_LO_RTL_MASK                               (0x0000F800U)
#define CSL_I2C_REVNB_LO_RTL_SHIFT                              (11U)
#define CSL_I2C_REVNB_LO_RTL_RESETVAL                           (0x00000000U)
#define CSL_I2C_REVNB_LO_RTL_MAX                                (0x0000001fU)

#define CSL_I2C_REVNB_LO_MAJOR_MASK                             (0x00000700U)
#define CSL_I2C_REVNB_LO_MAJOR_SHIFT                            (8U)
#define CSL_I2C_REVNB_LO_MAJOR_RESETVAL                         (0x00000000U)
#define CSL_I2C_REVNB_LO_MAJOR_MAX                              (0x00000007U)

#define CSL_I2C_REVNB_LO_RESETVAL                               (0x00000003U)

/* REVNB_HI */

#define CSL_I2C_REVNB_HI_SCHEME_MASK                            (0x0000C000U)
#define CSL_I2C_REVNB_HI_SCHEME_SHIFT                           (14U)
#define CSL_I2C_REVNB_HI_SCHEME_RESETVAL                        (0x00000001U)
#define CSL_I2C_REVNB_HI_SCHEME_MAX                             (0x00000003U)

#define CSL_I2C_REVNB_HI_FUNC_MASK                              (0x00000FFFU)
#define CSL_I2C_REVNB_HI_FUNC_SHIFT                             (0U)
#define CSL_I2C_REVNB_HI_FUNC_RESETVAL                          (0x00000040U)
#define CSL_I2C_REVNB_HI_FUNC_MAX                               (0x00000fffU)

#define CSL_I2C_REVNB_HI_RESETVAL                               (0x00005040U)

/* SYSC */

#define CSL_I2C_SYSC_AUTOIDLE_MASK                              (0x00000001U)
#define CSL_I2C_SYSC_AUTOIDLE_SHIFT                             (0U)
#define CSL_I2C_SYSC_AUTOIDLE_RESETVAL                          (0x00000001U)
#define CSL_I2C_SYSC_AUTOIDLE_DISABLE                           (0x00000000U)
#define CSL_I2C_SYSC_AUTOIDLE_ENABLE                            (0x00000001U)

#define CSL_I2C_SYSC_SRST_MASK                                  (0x00000002U)
#define CSL_I2C_SYSC_SRST_SHIFT                                 (1U)
#define CSL_I2C_SYSC_SRST_RESETVAL                              (0x00000000U)
#define CSL_I2C_SYSC_SRST_NMODE                                 (0x00000000U)
#define CSL_I2C_SYSC_SRST_RSTMODE                               (0x00000001U)

#define CSL_I2C_SYSC_ENAWAKEUP_MASK                             (0x00000004U)
#define CSL_I2C_SYSC_ENAWAKEUP_SHIFT                            (2U)
#define CSL_I2C_SYSC_ENAWAKEUP_RESETVAL                         (0x00000000U)
#define CSL_I2C_SYSC_ENAWAKEUP_DISABLE                          (0x00000000U)
#define CSL_I2C_SYSC_ENAWAKEUP_ENABLE                           (0x00000001U)

#define CSL_I2C_SYSC_IDLEMODE_MASK                              (0x00000018U)
#define CSL_I2C_SYSC_IDLEMODE_SHIFT                             (3U)
#define CSL_I2C_SYSC_IDLEMODE_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSC_IDLEMODE_FORCEIDLE                         (0x00000000U)
#define CSL_I2C_SYSC_IDLEMODE_SMARTIDLE_WAKEUP                  (0x00000003U)
#define CSL_I2C_SYSC_IDLEMODE_NOIDLE                            (0x00000001U)
#define CSL_I2C_SYSC_IDLEMODE_SMARTIDLE                         (0x00000002U)

#define CSL_I2C_SYSC_CLKACTIVITY_MASK                           (0x00000300U)
#define CSL_I2C_SYSC_CLKACTIVITY_SHIFT                          (8U)
#define CSL_I2C_SYSC_CLKACTIVITY_RESETVAL                       (0x00000000U)
#define CSL_I2C_SYSC_CLKACTIVITY_BOOTHOFF                       (0x00000000U)
#define CSL_I2C_SYSC_CLKACTIVITY_OCPON                          (0x00000001U)
#define CSL_I2C_SYSC_CLKACTIVITY_SYSON                          (0x00000002U)
#define CSL_I2C_SYSC_CLKACTIVITY_BOOTHON                        (0x00000003U)

#define CSL_I2C_SYSC_RESETVAL                                   (0x00000001U)

/* EOI */

#define CSL_I2C_EOI_LINE_NUMBER_MASK                            (0x00000001U)
#define CSL_I2C_EOI_LINE_NUMBER_SHIFT                           (0U)
#define CSL_I2C_EOI_LINE_NUMBER_RESETVAL                        (0x00000000U)
#define CSL_I2C_EOI_LINE_NUMBER_MAX                             (0x00000001U)

#define CSL_I2C_EOI_RESETVAL                                    (0x00000000U)

/* IRQSTATUS_RAW */

#define CSL_I2C_IRQSTATUS_RAW_AL_MASK                           (0x00000001U)
#define CSL_I2C_IRQSTATUS_RAW_AL_SHIFT                          (0U)
#define CSL_I2C_IRQSTATUS_RAW_AL_RESETVAL                       (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AL_CLEAR                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AL_SET                            (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_NACK_MASK                         (0x00000002U)
#define CSL_I2C_IRQSTATUS_RAW_NACK_SHIFT                        (1U)
#define CSL_I2C_IRQSTATUS_RAW_NACK_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_NACK_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_NACK_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_ARDY_MASK                         (0x00000004U)
#define CSL_I2C_IRQSTATUS_RAW_ARDY_SHIFT                        (2U)
#define CSL_I2C_IRQSTATUS_RAW_ARDY_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_ARDY_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_ARDY_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_RRDY_MASK                         (0x00000008U)
#define CSL_I2C_IRQSTATUS_RAW_RRDY_SHIFT                        (3U)
#define CSL_I2C_IRQSTATUS_RAW_RRDY_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_RRDY_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_RRDY_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_XRDY_MASK                         (0x00000010U)
#define CSL_I2C_IRQSTATUS_RAW_XRDY_SHIFT                        (4U)
#define CSL_I2C_IRQSTATUS_RAW_XRDY_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XRDY_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XRDY_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_GC_MASK                           (0x00000020U)
#define CSL_I2C_IRQSTATUS_RAW_GC_SHIFT                          (5U)
#define CSL_I2C_IRQSTATUS_RAW_GC_RESETVAL                       (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_GC_CLEAR                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_GC_SET                            (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_STC_MASK                          (0x00000040U)
#define CSL_I2C_IRQSTATUS_RAW_STC_SHIFT                         (6U)
#define CSL_I2C_IRQSTATUS_RAW_STC_RESETVAL                      (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_STC_CLEAR                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_STC_SET                           (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_AERR_MASK                         (0x00000080U)
#define CSL_I2C_IRQSTATUS_RAW_AERR_SHIFT                        (7U)
#define CSL_I2C_IRQSTATUS_RAW_AERR_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AERR_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AERR_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_BF_MASK                           (0x00000100U)
#define CSL_I2C_IRQSTATUS_RAW_BF_SHIFT                          (8U)
#define CSL_I2C_IRQSTATUS_RAW_BF_RESETVAL                       (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_BF_CLEAR                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_BF_SET                            (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_AAS_MASK                          (0x00000200U)
#define CSL_I2C_IRQSTATUS_RAW_AAS_SHIFT                         (9U)
#define CSL_I2C_IRQSTATUS_RAW_AAS_RESETVAL                      (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AAS_CLEAR                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_AAS_SET                           (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_XUDF_MASK                         (0x00000400U)
#define CSL_I2C_IRQSTATUS_RAW_XUDF_SHIFT                        (10U)
#define CSL_I2C_IRQSTATUS_RAW_XUDF_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XUDF_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XUDF_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_ROVR_MASK                         (0x00000800U)
#define CSL_I2C_IRQSTATUS_RAW_ROVR_SHIFT                        (11U)
#define CSL_I2C_IRQSTATUS_RAW_ROVR_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_ROVR_CLEAR                        (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_ROVR_SET                          (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_BB_MASK                           (0x00001000U)
#define CSL_I2C_IRQSTATUS_RAW_BB_SHIFT                          (12U)
#define CSL_I2C_IRQSTATUS_RAW_BB_RESETVAL                       (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_BB_CLEAR                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_BB_SET                            (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_RDR_MASK                          (0x00002000U)
#define CSL_I2C_IRQSTATUS_RAW_RDR_SHIFT                         (13U)
#define CSL_I2C_IRQSTATUS_RAW_RDR_RESETVAL                      (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_RDR_CLEAR                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_RDR_SET                           (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_XDR_MASK                          (0x00004000U)
#define CSL_I2C_IRQSTATUS_RAW_XDR_SHIFT                         (14U)
#define CSL_I2C_IRQSTATUS_RAW_XDR_RESETVAL                      (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XDR_CLEAR                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_RAW_XDR_SET                           (0x00000001U)

#define CSL_I2C_IRQSTATUS_RAW_RESETVAL                          (0x00000000U)

/* IRQSTATUS */

#define CSL_I2C_IRQSTATUS_AL_MASK                               (0x00000001U)
#define CSL_I2C_IRQSTATUS_AL_SHIFT                              (0U)
#define CSL_I2C_IRQSTATUS_AL_RESETVAL                           (0x00000000U)
#define CSL_I2C_IRQSTATUS_AL_CLEAR                              (0x00000000U)
#define CSL_I2C_IRQSTATUS_AL_SET                                (0x00000001U)

#define CSL_I2C_IRQSTATUS_NACK_MASK                             (0x00000002U)
#define CSL_I2C_IRQSTATUS_NACK_SHIFT                            (1U)
#define CSL_I2C_IRQSTATUS_NACK_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_NACK_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_NACK_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_ARDY_MASK                             (0x00000004U)
#define CSL_I2C_IRQSTATUS_ARDY_SHIFT                            (2U)
#define CSL_I2C_IRQSTATUS_ARDY_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_ARDY_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_ARDY_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_RRDY_MASK                             (0x00000008U)
#define CSL_I2C_IRQSTATUS_RRDY_SHIFT                            (3U)
#define CSL_I2C_IRQSTATUS_RRDY_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_RRDY_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_RRDY_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_XRDY_MASK                             (0x00000010U)
#define CSL_I2C_IRQSTATUS_XRDY_SHIFT                            (4U)
#define CSL_I2C_IRQSTATUS_XRDY_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_XRDY_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_XRDY_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_GC_MASK                               (0x00000020U)
#define CSL_I2C_IRQSTATUS_GC_SHIFT                              (5U)
#define CSL_I2C_IRQSTATUS_GC_RESETVAL                           (0x00000000U)
#define CSL_I2C_IRQSTATUS_GC_CLEAR                              (0x00000000U)
#define CSL_I2C_IRQSTATUS_GC_SET                                (0x00000001U)

#define CSL_I2C_IRQSTATUS_STC_MASK                              (0x00000040U)
#define CSL_I2C_IRQSTATUS_STC_SHIFT                             (6U)
#define CSL_I2C_IRQSTATUS_STC_RESETVAL                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_STC_CLEAR                             (0x00000000U)
#define CSL_I2C_IRQSTATUS_STC_SET                               (0x00000001U)

#define CSL_I2C_IRQSTATUS_AERR_MASK                             (0x00000080U)
#define CSL_I2C_IRQSTATUS_AERR_SHIFT                            (7U)
#define CSL_I2C_IRQSTATUS_AERR_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_AERR_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_AERR_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_BF_MASK                               (0x00000100U)
#define CSL_I2C_IRQSTATUS_BF_SHIFT                              (8U)
#define CSL_I2C_IRQSTATUS_BF_RESETVAL                           (0x00000000U)
#define CSL_I2C_IRQSTATUS_BF_CLEAR                              (0x00000000U)
#define CSL_I2C_IRQSTATUS_BF_SET                                (0x00000001U)

#define CSL_I2C_IRQSTATUS_AAS_MASK                              (0x00000200U)
#define CSL_I2C_IRQSTATUS_AAS_SHIFT                             (9U)
#define CSL_I2C_IRQSTATUS_AAS_RESETVAL                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_AAS_CLEAR                             (0x00000000U)
#define CSL_I2C_IRQSTATUS_AAS_SET                               (0x00000001U)

#define CSL_I2C_IRQSTATUS_XUDF_MASK                             (0x00000400U)
#define CSL_I2C_IRQSTATUS_XUDF_SHIFT                            (10U)
#define CSL_I2C_IRQSTATUS_XUDF_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_XUDF_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_XUDF_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_ROVR_MASK                             (0x00000800U)
#define CSL_I2C_IRQSTATUS_ROVR_SHIFT                            (11U)
#define CSL_I2C_IRQSTATUS_ROVR_RESETVAL                         (0x00000000U)
#define CSL_I2C_IRQSTATUS_ROVR_CLEAR                            (0x00000000U)
#define CSL_I2C_IRQSTATUS_ROVR_SET                              (0x00000001U)

#define CSL_I2C_IRQSTATUS_BB_MASK                               (0x00001000U)
#define CSL_I2C_IRQSTATUS_BB_SHIFT                              (12U)
#define CSL_I2C_IRQSTATUS_BB_RESETVAL                           (0x00000000U)
#define CSL_I2C_IRQSTATUS_BB_CLEAR                              (0x00000000U)
#define CSL_I2C_IRQSTATUS_BB_SET                                (0x00000001U)

#define CSL_I2C_IRQSTATUS_RDR_MASK                              (0x00002000U)
#define CSL_I2C_IRQSTATUS_RDR_SHIFT                             (13U)
#define CSL_I2C_IRQSTATUS_RDR_RESETVAL                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_RDR_CLEAR                             (0x00000000U)
#define CSL_I2C_IRQSTATUS_RDR_SET                               (0x00000001U)

#define CSL_I2C_IRQSTATUS_XDR_MASK                              (0x00004000U)
#define CSL_I2C_IRQSTATUS_XDR_SHIFT                             (14U)
#define CSL_I2C_IRQSTATUS_XDR_RESETVAL                          (0x00000000U)
#define CSL_I2C_IRQSTATUS_XDR_CLEAR                             (0x00000000U)
#define CSL_I2C_IRQSTATUS_XDR_SET                               (0x00000001U)

#define CSL_I2C_IRQSTATUS_RESETVAL                              (0x00000000U)

/* IRQENABLE_SET */

#define CSL_I2C_IRQENABLE_SET_AL_IE_MASK                        (0x00000001U)
#define CSL_I2C_IRQENABLE_SET_AL_IE_SHIFT                       (0U)
#define CSL_I2C_IRQENABLE_SET_AL_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_AL_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_AL_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_NACK_IE_MASK                      (0x00000002U)
#define CSL_I2C_IRQENABLE_SET_NACK_IE_SHIFT                     (1U)
#define CSL_I2C_IRQENABLE_SET_NACK_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_NACK_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_NACK_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_ARDY_IE_MASK                      (0x00000004U)
#define CSL_I2C_IRQENABLE_SET_ARDY_IE_SHIFT                     (2U)
#define CSL_I2C_IRQENABLE_SET_ARDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ARDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ARDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_RRDY_IE_MASK                      (0x00000008U)
#define CSL_I2C_IRQENABLE_SET_RRDY_IE_SHIFT                     (3U)
#define CSL_I2C_IRQENABLE_SET_RRDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_RRDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_RRDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_XRDY_IE_MASK                      (0x00000010U)
#define CSL_I2C_IRQENABLE_SET_XRDY_IE_SHIFT                     (4U)
#define CSL_I2C_IRQENABLE_SET_XRDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XRDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XRDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_GC_IE_MASK                        (0x00000020U)
#define CSL_I2C_IRQENABLE_SET_GC_IE_SHIFT                       (5U)
#define CSL_I2C_IRQENABLE_SET_GC_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_GC_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_GC_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_STC_IE_MASK                       (0x00000040U)
#define CSL_I2C_IRQENABLE_SET_STC_IE_SHIFT                      (6U)
#define CSL_I2C_IRQENABLE_SET_STC_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_STC_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_STC_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_AERR_IE_MASK                      (0x00000080U)
#define CSL_I2C_IRQENABLE_SET_AERR_IE_SHIFT                     (7U)
#define CSL_I2C_IRQENABLE_SET_AERR_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_AERR_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_AERR_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_BF_IE_MASK                        (0x00000100U)
#define CSL_I2C_IRQENABLE_SET_BF_IE_SHIFT                       (8U)
#define CSL_I2C_IRQENABLE_SET_BF_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_BF_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_BF_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_ASS_IE_MASK                       (0x00000200U)
#define CSL_I2C_IRQENABLE_SET_ASS_IE_SHIFT                      (9U)
#define CSL_I2C_IRQENABLE_SET_ASS_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ASS_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ASS_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_XUDF_MASK                         (0x00000400U)
#define CSL_I2C_IRQENABLE_SET_XUDF_SHIFT                        (10U)
#define CSL_I2C_IRQENABLE_SET_XUDF_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XUDF_DISABLE                      (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XUDF_ENABLE                       (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_ROVR_MASK                         (0x00000800U)
#define CSL_I2C_IRQENABLE_SET_ROVR_SHIFT                        (11U)
#define CSL_I2C_IRQENABLE_SET_ROVR_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ROVR_DISABLE                      (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_ROVR_ENABLE                       (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_RDR_IE_MASK                       (0x00002000U)
#define CSL_I2C_IRQENABLE_SET_RDR_IE_SHIFT                      (13U)
#define CSL_I2C_IRQENABLE_SET_RDR_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_RDR_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_RDR_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_XDR_IE_MASK                       (0x00004000U)
#define CSL_I2C_IRQENABLE_SET_XDR_IE_SHIFT                      (14U)
#define CSL_I2C_IRQENABLE_SET_XDR_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XDR_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_SET_XDR_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_SET_RESETVAL                          (0x00000000U)

/* IRQENABLE_CLR */

#define CSL_I2C_IRQENABLE_CLR_AL_IE_MASK                        (0x00000001U)
#define CSL_I2C_IRQENABLE_CLR_AL_IE_SHIFT                       (0U)
#define CSL_I2C_IRQENABLE_CLR_AL_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_AL_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_AL_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_NACK_IE_MASK                      (0x00000002U)
#define CSL_I2C_IRQENABLE_CLR_NACK_IE_SHIFT                     (1U)
#define CSL_I2C_IRQENABLE_CLR_NACK_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_NACK_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_NACK_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_ARDY_IE_MASK                      (0x00000004U)
#define CSL_I2C_IRQENABLE_CLR_ARDY_IE_SHIFT                     (2U)
#define CSL_I2C_IRQENABLE_CLR_ARDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ARDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ARDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_RRDY_IE_MASK                      (0x00000008U)
#define CSL_I2C_IRQENABLE_CLR_RRDY_IE_SHIFT                     (3U)
#define CSL_I2C_IRQENABLE_CLR_RRDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_RRDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_RRDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_XRDY_IE_MASK                      (0x00000010U)
#define CSL_I2C_IRQENABLE_CLR_XRDY_IE_SHIFT                     (4U)
#define CSL_I2C_IRQENABLE_CLR_XRDY_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XRDY_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XRDY_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_GC_IE_MASK                        (0x00000020U)
#define CSL_I2C_IRQENABLE_CLR_GC_IE_SHIFT                       (5U)
#define CSL_I2C_IRQENABLE_CLR_GC_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_GC_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_GC_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_STC_IE_MASK                       (0x00000040U)
#define CSL_I2C_IRQENABLE_CLR_STC_IE_SHIFT                      (6U)
#define CSL_I2C_IRQENABLE_CLR_STC_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_STC_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_STC_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_AERR_IE_MASK                      (0x00000080U)
#define CSL_I2C_IRQENABLE_CLR_AERR_IE_SHIFT                     (7U)
#define CSL_I2C_IRQENABLE_CLR_AERR_IE_RESETVAL                  (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_AERR_IE_DISABLE                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_AERR_IE_ENABLE                    (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_BF_IE_MASK                        (0x00000100U)
#define CSL_I2C_IRQENABLE_CLR_BF_IE_SHIFT                       (8U)
#define CSL_I2C_IRQENABLE_CLR_BF_IE_RESETVAL                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_BF_IE_DISABLE                     (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_BF_IE_ENABLE                      (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_ASS_IE_MASK                       (0x00000200U)
#define CSL_I2C_IRQENABLE_CLR_ASS_IE_SHIFT                      (9U)
#define CSL_I2C_IRQENABLE_CLR_ASS_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ASS_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ASS_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_XUDF_MASK                         (0x00000400U)
#define CSL_I2C_IRQENABLE_CLR_XUDF_SHIFT                        (10U)
#define CSL_I2C_IRQENABLE_CLR_XUDF_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XUDF_DISABLE                      (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XUDF_ENABLE                       (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_ROVR_MASK                         (0x00000800U)
#define CSL_I2C_IRQENABLE_CLR_ROVR_SHIFT                        (11U)
#define CSL_I2C_IRQENABLE_CLR_ROVR_RESETVAL                     (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ROVR_DISABLE                      (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_ROVR_ENABLE                       (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_RDR_IE_MASK                       (0x00002000U)
#define CSL_I2C_IRQENABLE_CLR_RDR_IE_SHIFT                      (13U)
#define CSL_I2C_IRQENABLE_CLR_RDR_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_RDR_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_RDR_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_XDR_IE_MASK                       (0x00004000U)
#define CSL_I2C_IRQENABLE_CLR_XDR_IE_SHIFT                      (14U)
#define CSL_I2C_IRQENABLE_CLR_XDR_IE_RESETVAL                   (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XDR_IE_DISABLE                    (0x00000000U)
#define CSL_I2C_IRQENABLE_CLR_XDR_IE_ENABLE                     (0x00000001U)

#define CSL_I2C_IRQENABLE_CLR_RESETVAL                          (0x00000000U)

/* WE */

#define CSL_I2C_WE_AL_MASK                                      (0x00000001U)
#define CSL_I2C_WE_AL_SHIFT                                     (0U)
#define CSL_I2C_WE_AL_RESETVAL                                  (0x00000000U)
#define CSL_I2C_WE_AL_ENABLE                                    (0x00000001U)
#define CSL_I2C_WE_AL_DISABLE                                   (0x00000000U)

#define CSL_I2C_WE_NACK_MASK                                    (0x00000002U)
#define CSL_I2C_WE_NACK_SHIFT                                   (1U)
#define CSL_I2C_WE_NACK_RESETVAL                                (0x00000000U)
#define CSL_I2C_WE_NACK_DISABLE                                 (0x00000000U)
#define CSL_I2C_WE_NACK_ENABLE                                  (0x00000001U)

#define CSL_I2C_WE_ARDY_MASK                                    (0x00000004U)
#define CSL_I2C_WE_ARDY_SHIFT                                   (2U)
#define CSL_I2C_WE_ARDY_RESETVAL                                (0x00000000U)
#define CSL_I2C_WE_ARDY_ENABLE                                  (0x00000001U)
#define CSL_I2C_WE_ARDY_DISABLE                                 (0x00000000U)

#define CSL_I2C_WE_DRDY_MASK                                    (0x00000008U)
#define CSL_I2C_WE_DRDY_SHIFT                                   (3U)
#define CSL_I2C_WE_DRDY_RESETVAL                                (0x00000000U)
#define CSL_I2C_WE_DRDY_DISABLE                                 (0x00000000U)
#define CSL_I2C_WE_DRDY_ENABLE                                  (0x00000001U)

#define CSL_I2C_WE_GC_MASK                                      (0x00000020U)
#define CSL_I2C_WE_GC_SHIFT                                     (5U)
#define CSL_I2C_WE_GC_RESETVAL                                  (0x00000000U)
#define CSL_I2C_WE_GC_DISABLE                                   (0x00000000U)
#define CSL_I2C_WE_GC_ENABLE                                    (0x00000001U)

#define CSL_I2C_WE_STC_MASK                                     (0x00000040U)
#define CSL_I2C_WE_STC_SHIFT                                    (6U)
#define CSL_I2C_WE_STC_RESETVAL                                 (0x00000000U)
#define CSL_I2C_WE_STC_DISABLE                                  (0x00000000U)
#define CSL_I2C_WE_STC_ENABLE                                   (0x00000001U)

#define CSL_I2C_WE_BF_MASK                                      (0x00000100U)
#define CSL_I2C_WE_BF_SHIFT                                     (8U)
#define CSL_I2C_WE_BF_RESETVAL                                  (0x00000000U)
#define CSL_I2C_WE_BF_DISABLE                                   (0x00000000U)
#define CSL_I2C_WE_BF_ENABLE                                    (0x00000001U)

#define CSL_I2C_WE_AAS_MASK                                     (0x00000200U)
#define CSL_I2C_WE_AAS_SHIFT                                    (9U)
#define CSL_I2C_WE_AAS_RESETVAL                                 (0x00000000U)
#define CSL_I2C_WE_AAS_DISABLE                                  (0x00000000U)
#define CSL_I2C_WE_AAS_ENABLE                                   (0x00000001U)

#define CSL_I2C_WE_XUDF_MASK                                    (0x00000400U)
#define CSL_I2C_WE_XUDF_SHIFT                                   (10U)
#define CSL_I2C_WE_XUDF_RESETVAL                                (0x00000000U)
#define CSL_I2C_WE_XUDF_DISABLE                                 (0x00000000U)
#define CSL_I2C_WE_XUDF_ENABLE                                  (0x00000001U)

#define CSL_I2C_WE_ROVR_MASK                                    (0x00000800U)
#define CSL_I2C_WE_ROVR_SHIFT                                   (11U)
#define CSL_I2C_WE_ROVR_RESETVAL                                (0x00000000U)
#define CSL_I2C_WE_ROVR_DISABLE                                 (0x00000000U)
#define CSL_I2C_WE_ROVR_ENABLE                                  (0x00000001U)

#define CSL_I2C_WE_RDR_MASK                                     (0x00002000U)
#define CSL_I2C_WE_RDR_SHIFT                                    (13U)
#define CSL_I2C_WE_RDR_RESETVAL                                 (0x00000000U)
#define CSL_I2C_WE_RDR_DISABLE                                  (0x00000000U)
#define CSL_I2C_WE_RDR_ENABLE                                   (0x00000001U)

#define CSL_I2C_WE_XDR_MASK                                     (0x00004000U)
#define CSL_I2C_WE_XDR_SHIFT                                    (14U)
#define CSL_I2C_WE_XDR_RESETVAL                                 (0x00000000U)
#define CSL_I2C_WE_XDR_DISABLE                                  (0x00000000U)
#define CSL_I2C_WE_XDR_ENABLE                                   (0x00000001U)

#define CSL_I2C_WE_RESETVAL                                     (0x00000000U)

/* DMARXENABLE_SET */

#define CSL_I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_MASK           (0x00000001U)
#define CSL_I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_SHIFT          (0U)
#define CSL_I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_RESETVAL       (0x00000000U)
#define CSL_I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_MAX            (0x00000001U)

#define CSL_I2C_DMARXENABLE_SET_RESETVAL                        (0x00000000U)

/* DMATXENABLE_SET */

#define CSL_I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_MASK           (0x00000001U)
#define CSL_I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_SHIFT          (0U)
#define CSL_I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_RESETVAL       (0x00000000U)
#define CSL_I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_MAX            (0x00000001U)

#define CSL_I2C_DMATXENABLE_SET_RESETVAL                        (0x00000000U)

/* DMARXENABLE_CLR */

#define CSL_I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_MASK         (0x00000001U)
#define CSL_I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_SHIFT        (0U)
#define CSL_I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_RESETVAL     (0x00000000U)
#define CSL_I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_MAX          (0x00000001U)

#define CSL_I2C_DMARXENABLE_CLR_RESETVAL                        (0x00000000U)

/* DMATXENABLE_CLR */

#define CSL_I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_MASK         (0x00000001U)
#define CSL_I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_SHIFT        (0U)
#define CSL_I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_RESETVAL     (0x00000000U)
#define CSL_I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_MAX          (0x00000001U)

#define CSL_I2C_DMATXENABLE_CLR_RESETVAL                        (0x00000000U)

/* DMARXWAKE_EN */

#define CSL_I2C_DMARXWAKE_EN_AL_MASK                            (0x00000001U)
#define CSL_I2C_DMARXWAKE_EN_AL_SHIFT                           (0U)
#define CSL_I2C_DMARXWAKE_EN_AL_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_AL_ENABLE                          (0x00000001U)
#define CSL_I2C_DMARXWAKE_EN_AL_DISABLE                         (0x00000000U)

#define CSL_I2C_DMARXWAKE_EN_NACK_MASK                          (0x00000002U)
#define CSL_I2C_DMARXWAKE_EN_NACK_SHIFT                         (1U)
#define CSL_I2C_DMARXWAKE_EN_NACK_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_NACK_DISABLE                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_NACK_ENABLE                        (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_ARDY_MASK                          (0x00000004U)
#define CSL_I2C_DMARXWAKE_EN_ARDY_SHIFT                         (2U)
#define CSL_I2C_DMARXWAKE_EN_ARDY_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_ARDY_ENABLE                        (0x00000001U)
#define CSL_I2C_DMARXWAKE_EN_ARDY_DISABLE                       (0x00000000U)

#define CSL_I2C_DMARXWAKE_EN_DRDY_MASK                          (0x00000008U)
#define CSL_I2C_DMARXWAKE_EN_DRDY_SHIFT                         (3U)
#define CSL_I2C_DMARXWAKE_EN_DRDY_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_DRDY_DISABLE                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_DRDY_ENABLE                        (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_GC_MASK                            (0x00000020U)
#define CSL_I2C_DMARXWAKE_EN_GC_SHIFT                           (5U)
#define CSL_I2C_DMARXWAKE_EN_GC_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_GC_DISABLE                         (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_GC_ENABLE                          (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_STC_MASK                           (0x00000040U)
#define CSL_I2C_DMARXWAKE_EN_STC_SHIFT                          (6U)
#define CSL_I2C_DMARXWAKE_EN_STC_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_STC_DISABLE                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_STC_ENABLE                         (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_BF_MASK                            (0x00000100U)
#define CSL_I2C_DMARXWAKE_EN_BF_SHIFT                           (8U)
#define CSL_I2C_DMARXWAKE_EN_BF_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_BF_DISABLE                         (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_BF_ENABLE                          (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_AAS_MASK                           (0x00000200U)
#define CSL_I2C_DMARXWAKE_EN_AAS_SHIFT                          (9U)
#define CSL_I2C_DMARXWAKE_EN_AAS_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_AAS_DISABLE                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_AAS_ENABLE                         (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_XUDF_MASK                          (0x00000400U)
#define CSL_I2C_DMARXWAKE_EN_XUDF_SHIFT                         (10U)
#define CSL_I2C_DMARXWAKE_EN_XUDF_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_XUDF_DISABLE                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_XUDF_ENABLE                        (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_ROVR_MASK                          (0x00000800U)
#define CSL_I2C_DMARXWAKE_EN_ROVR_SHIFT                         (11U)
#define CSL_I2C_DMARXWAKE_EN_ROVR_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_ROVR_DISABLE                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_ROVR_ENABLE                        (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_RDR_MASK                           (0x00002000U)
#define CSL_I2C_DMARXWAKE_EN_RDR_SHIFT                          (13U)
#define CSL_I2C_DMARXWAKE_EN_RDR_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_RDR_DISABLE                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_RDR_ENABLE                         (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_XDR_MASK                           (0x00004000U)
#define CSL_I2C_DMARXWAKE_EN_XDR_SHIFT                          (14U)
#define CSL_I2C_DMARXWAKE_EN_XDR_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_XDR_DISABLE                        (0x00000000U)
#define CSL_I2C_DMARXWAKE_EN_XDR_ENABLE                         (0x00000001U)

#define CSL_I2C_DMARXWAKE_EN_RESETVAL                           (0x00000000U)

/* DMATXWAKE_EN */

#define CSL_I2C_DMATXWAKE_EN_AL_MASK                            (0x00000001U)
#define CSL_I2C_DMATXWAKE_EN_AL_SHIFT                           (0U)
#define CSL_I2C_DMATXWAKE_EN_AL_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_AL_ENABLE                          (0x00000001U)
#define CSL_I2C_DMATXWAKE_EN_AL_DISABLE                         (0x00000000U)

#define CSL_I2C_DMATXWAKE_EN_NACK_MASK                          (0x00000002U)
#define CSL_I2C_DMATXWAKE_EN_NACK_SHIFT                         (1U)
#define CSL_I2C_DMATXWAKE_EN_NACK_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_NACK_DISABLE                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_NACK_ENABLE                        (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_ARDY_MASK                          (0x00000004U)
#define CSL_I2C_DMATXWAKE_EN_ARDY_SHIFT                         (2U)
#define CSL_I2C_DMATXWAKE_EN_ARDY_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_ARDY_ENABLE                        (0x00000001U)
#define CSL_I2C_DMATXWAKE_EN_ARDY_DISABLE                       (0x00000000U)

#define CSL_I2C_DMATXWAKE_EN_DRDY_MASK                          (0x00000008U)
#define CSL_I2C_DMATXWAKE_EN_DRDY_SHIFT                         (3U)
#define CSL_I2C_DMATXWAKE_EN_DRDY_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_DRDY_DISABLE                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_DRDY_ENABLE                        (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_GC_MASK                            (0x00000020U)
#define CSL_I2C_DMATXWAKE_EN_GC_SHIFT                           (5U)
#define CSL_I2C_DMATXWAKE_EN_GC_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_GC_DISABLE                         (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_GC_ENABLE                          (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_STC_MASK                           (0x00000040U)
#define CSL_I2C_DMATXWAKE_EN_STC_SHIFT                          (6U)
#define CSL_I2C_DMATXWAKE_EN_STC_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_STC_DISABLE                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_STC_ENABLE                         (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_BF_MASK                            (0x00000100U)
#define CSL_I2C_DMATXWAKE_EN_BF_SHIFT                           (8U)
#define CSL_I2C_DMATXWAKE_EN_BF_RESETVAL                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_BF_DISABLE                         (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_BF_ENABLE                          (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_AAS_MASK                           (0x00000200U)
#define CSL_I2C_DMATXWAKE_EN_AAS_SHIFT                          (9U)
#define CSL_I2C_DMATXWAKE_EN_AAS_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_AAS_DISABLE                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_AAS_ENABLE                         (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_XUDF_MASK                          (0x00000400U)
#define CSL_I2C_DMATXWAKE_EN_XUDF_SHIFT                         (10U)
#define CSL_I2C_DMATXWAKE_EN_XUDF_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_XUDF_DISABLE                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_XUDF_ENABLE                        (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_ROVR_MASK                          (0x00000800U)
#define CSL_I2C_DMATXWAKE_EN_ROVR_SHIFT                         (11U)
#define CSL_I2C_DMATXWAKE_EN_ROVR_RESETVAL                      (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_ROVR_DISABLE                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_ROVR_ENABLE                        (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_RDR_MASK                           (0x00002000U)
#define CSL_I2C_DMATXWAKE_EN_RDR_SHIFT                          (13U)
#define CSL_I2C_DMATXWAKE_EN_RDR_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_RDR_DISABLE                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_RDR_ENABLE                         (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_XDR_MASK                           (0x00004000U)
#define CSL_I2C_DMATXWAKE_EN_XDR_SHIFT                          (14U)
#define CSL_I2C_DMATXWAKE_EN_XDR_RESETVAL                       (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_XDR_DISABLE                        (0x00000000U)
#define CSL_I2C_DMATXWAKE_EN_XDR_ENABLE                         (0x00000001U)

#define CSL_I2C_DMATXWAKE_EN_RESETVAL                           (0x00000000U)

/* IE */

#define CSL_I2C_IE_XUDF_MASK                                    (0x00000400U)
#define CSL_I2C_IE_XUDF_SHIFT                                   (10U)
#define CSL_I2C_IE_XUDF_RESETVAL                                (0x00000000U)
#define CSL_I2C_IE_XUDF_DISABLE                                 (0x00000000U)
#define CSL_I2C_IE_XUDF_ENABLE                                  (0x00000001U)

#define CSL_I2C_IE_ROVR_MASK                                    (0x00000800U)
#define CSL_I2C_IE_ROVR_SHIFT                                   (11U)
#define CSL_I2C_IE_ROVR_RESETVAL                                (0x00000000U)
#define CSL_I2C_IE_ROVR_DISABLE                                 (0x00000000U)
#define CSL_I2C_IE_ROVR_ENABLE                                  (0x00000001U)

#define CSL_I2C_IE_AL_IE_MASK                                   (0x00000001U)
#define CSL_I2C_IE_AL_IE_SHIFT                                  (0U)
#define CSL_I2C_IE_AL_IE_RESETVAL                               (0x00000000U)
#define CSL_I2C_IE_AL_IE_DISABLE                                (0x00000000U)
#define CSL_I2C_IE_AL_IE_ENABLE                                 (0x00000001U)

#define CSL_I2C_IE_NACK_IE_MASK                                 (0x00000002U)
#define CSL_I2C_IE_NACK_IE_SHIFT                                (1U)
#define CSL_I2C_IE_NACK_IE_RESETVAL                             (0x00000000U)
#define CSL_I2C_IE_NACK_IE_DISABLE                              (0x00000000U)
#define CSL_I2C_IE_NACK_IE_ENABLE                               (0x00000001U)

#define CSL_I2C_IE_ARDY_IE_MASK                                 (0x00000004U)
#define CSL_I2C_IE_ARDY_IE_SHIFT                                (2U)
#define CSL_I2C_IE_ARDY_IE_RESETVAL                             (0x00000000U)
#define CSL_I2C_IE_ARDY_IE_DISABLE                              (0x00000000U)
#define CSL_I2C_IE_ARDY_IE_ENABLE                               (0x00000001U)

#define CSL_I2C_IE_RRDY_IE_MASK                                 (0x00000008U)
#define CSL_I2C_IE_RRDY_IE_SHIFT                                (3U)
#define CSL_I2C_IE_RRDY_IE_RESETVAL                             (0x00000000U)
#define CSL_I2C_IE_RRDY_IE_DISABLE                              (0x00000000U)
#define CSL_I2C_IE_RRDY_IE_ENABLE                               (0x00000001U)

#define CSL_I2C_IE_XRDY_IE_MASK                                 (0x00000010U)
#define CSL_I2C_IE_XRDY_IE_SHIFT                                (4U)
#define CSL_I2C_IE_XRDY_IE_RESETVAL                             (0x00000000U)
#define CSL_I2C_IE_XRDY_IE_DISABLE                              (0x00000000U)
#define CSL_I2C_IE_XRDY_IE_ENABLE                               (0x00000001U)

#define CSL_I2C_IE_GC_IE_MASK                                   (0x00000020U)
#define CSL_I2C_IE_GC_IE_SHIFT                                  (5U)
#define CSL_I2C_IE_GC_IE_RESETVAL                               (0x00000000U)
#define CSL_I2C_IE_GC_IE_DISABLE                                (0x00000000U)
#define CSL_I2C_IE_GC_IE_ENABLE                                 (0x00000001U)

#define CSL_I2C_IE_STC_IE_MASK                                  (0x00000040U)
#define CSL_I2C_IE_STC_IE_SHIFT                                 (6U)
#define CSL_I2C_IE_STC_IE_RESETVAL                              (0x00000000U)
#define CSL_I2C_IE_STC_IE_DISABLE                               (0x00000000U)
#define CSL_I2C_IE_STC_IE_ENABLE                                (0x00000001U)

#define CSL_I2C_IE_AERR_IE_MASK                                 (0x00000080U)
#define CSL_I2C_IE_AERR_IE_SHIFT                                (7U)
#define CSL_I2C_IE_AERR_IE_RESETVAL                             (0x00000000U)
#define CSL_I2C_IE_AERR_IE_DISABLE                              (0x00000000U)
#define CSL_I2C_IE_AERR_IE_ENABLE                               (0x00000001U)

#define CSL_I2C_IE_BF_IE_MASK                                   (0x00000100U)
#define CSL_I2C_IE_BF_IE_SHIFT                                  (8U)
#define CSL_I2C_IE_BF_IE_RESETVAL                               (0x00000000U)
#define CSL_I2C_IE_BF_IE_DISABLE                                (0x00000000U)
#define CSL_I2C_IE_BF_IE_ENABLE                                 (0x00000001U)

#define CSL_I2C_IE_ASS_IE_MASK                                  (0x00000200U)
#define CSL_I2C_IE_ASS_IE_SHIFT                                 (9U)
#define CSL_I2C_IE_ASS_IE_RESETVAL                              (0x00000000U)
#define CSL_I2C_IE_ASS_IE_DISABLE                               (0x00000000U)
#define CSL_I2C_IE_ASS_IE_ENABLE                                (0x00000001U)

#define CSL_I2C_IE_RDR_IE_MASK                                  (0x00002000U)
#define CSL_I2C_IE_RDR_IE_SHIFT                                 (13U)
#define CSL_I2C_IE_RDR_IE_RESETVAL                              (0x00000000U)
#define CSL_I2C_IE_RDR_IE_DISABLE                               (0x00000000U)
#define CSL_I2C_IE_RDR_IE_ENABLE                                (0x00000001U)

#define CSL_I2C_IE_XDR_IE_MASK                                  (0x00004000U)
#define CSL_I2C_IE_XDR_IE_SHIFT                                 (14U)
#define CSL_I2C_IE_XDR_IE_RESETVAL                              (0x00000000U)
#define CSL_I2C_IE_XDR_IE_DISABLE                               (0x00000000U)
#define CSL_I2C_IE_XDR_IE_ENABLE                                (0x00000001U)

#define CSL_I2C_IE_RESETVAL                                     (0x00000000U)

/* STAT */

#define CSL_I2C_STAT_AL_MASK                                    (0x00000001U)
#define CSL_I2C_STAT_AL_SHIFT                                   (0U)
#define CSL_I2C_STAT_AL_RESETVAL                                (0x00000000U)
#define CSL_I2C_STAT_AL_CLEAR                                   (0x00000000U)
#define CSL_I2C_STAT_AL_SET                                     (0x00000001U)

#define CSL_I2C_STAT_NACK_MASK                                  (0x00000002U)
#define CSL_I2C_STAT_NACK_SHIFT                                 (1U)
#define CSL_I2C_STAT_NACK_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_NACK_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_NACK_SET                                   (0x00000001U)

#define CSL_I2C_STAT_ARDY_MASK                                  (0x00000004U)
#define CSL_I2C_STAT_ARDY_SHIFT                                 (2U)
#define CSL_I2C_STAT_ARDY_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_ARDY_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_ARDY_SET                                   (0x00000001U)

#define CSL_I2C_STAT_RRDY_MASK                                  (0x00000008U)
#define CSL_I2C_STAT_RRDY_SHIFT                                 (3U)
#define CSL_I2C_STAT_RRDY_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_RRDY_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_RRDY_SET                                   (0x00000001U)

#define CSL_I2C_STAT_XRDY_MASK                                  (0x00000010U)
#define CSL_I2C_STAT_XRDY_SHIFT                                 (4U)
#define CSL_I2C_STAT_XRDY_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_XRDY_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_XRDY_SET                                   (0x00000001U)

#define CSL_I2C_STAT_GC_MASK                                    (0x00000020U)
#define CSL_I2C_STAT_GC_SHIFT                                   (5U)
#define CSL_I2C_STAT_GC_RESETVAL                                (0x00000000U)
#define CSL_I2C_STAT_GC_CLEAR                                   (0x00000000U)
#define CSL_I2C_STAT_GC_SET                                     (0x00000001U)

#define CSL_I2C_STAT_STC_MASK                                   (0x00000040U)
#define CSL_I2C_STAT_STC_SHIFT                                  (6U)
#define CSL_I2C_STAT_STC_RESETVAL                               (0x00000000U)
#define CSL_I2C_STAT_STC_CLEAR                                  (0x00000000U)
#define CSL_I2C_STAT_STC_SET                                    (0x00000001U)

#define CSL_I2C_STAT_AERR_MASK                                  (0x00000080U)
#define CSL_I2C_STAT_AERR_SHIFT                                 (7U)
#define CSL_I2C_STAT_AERR_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_AERR_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_AERR_SET                                   (0x00000001U)

#define CSL_I2C_STAT_BF_MASK                                    (0x00000100U)
#define CSL_I2C_STAT_BF_SHIFT                                   (8U)
#define CSL_I2C_STAT_BF_RESETVAL                                (0x00000000U)
#define CSL_I2C_STAT_BF_CLEAR                                   (0x00000000U)
#define CSL_I2C_STAT_BF_SET                                     (0x00000001U)

#define CSL_I2C_STAT_AAS_MASK                                   (0x00000200U)
#define CSL_I2C_STAT_AAS_SHIFT                                  (9U)
#define CSL_I2C_STAT_AAS_RESETVAL                               (0x00000000U)
#define CSL_I2C_STAT_AAS_CLEAR                                  (0x00000000U)
#define CSL_I2C_STAT_AAS_SET                                    (0x00000001U)

#define CSL_I2C_STAT_XUDF_MASK                                  (0x00000400U)
#define CSL_I2C_STAT_XUDF_SHIFT                                 (10U)
#define CSL_I2C_STAT_XUDF_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_XUDF_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_XUDF_SET                                   (0x00000001U)

#define CSL_I2C_STAT_ROVR_MASK                                  (0x00000800U)
#define CSL_I2C_STAT_ROVR_SHIFT                                 (11U)
#define CSL_I2C_STAT_ROVR_RESETVAL                              (0x00000000U)
#define CSL_I2C_STAT_ROVR_CLEAR                                 (0x00000000U)
#define CSL_I2C_STAT_ROVR_SET                                   (0x00000001U)

#define CSL_I2C_STAT_BB_MASK                                    (0x00001000U)
#define CSL_I2C_STAT_BB_SHIFT                                   (12U)
#define CSL_I2C_STAT_BB_RESETVAL                                (0x00000000U)
#define CSL_I2C_STAT_BB_CLEAR                                   (0x00000000U)
#define CSL_I2C_STAT_BB_SET                                     (0x00000001U)

#define CSL_I2C_STAT_RDR_MASK                                   (0x00002000U)
#define CSL_I2C_STAT_RDR_SHIFT                                  (13U)
#define CSL_I2C_STAT_RDR_RESETVAL                               (0x00000000U)
#define CSL_I2C_STAT_RDR_CLEAR                                  (0x00000000U)
#define CSL_I2C_STAT_RDR_SET                                    (0x00000001U)

#define CSL_I2C_STAT_XDR_MASK                                   (0x00004000U)
#define CSL_I2C_STAT_XDR_SHIFT                                  (14U)
#define CSL_I2C_STAT_XDR_RESETVAL                               (0x00000000U)
#define CSL_I2C_STAT_XDR_CLEAR                                  (0x00000000U)
#define CSL_I2C_STAT_XDR_SET                                    (0x00000001U)

#define CSL_I2C_STAT_RESETVAL                                   (0x00000000U)

/* SYSS */

#define CSL_I2C_SYSS_RDONE_MASK                                 (0x00000001U)
#define CSL_I2C_SYSS_RDONE_SHIFT                                (0U)
#define CSL_I2C_SYSS_RDONE_RESETVAL                             (0x00000001U)
#define CSL_I2C_SYSS_RDONE_RSTONGOING                           (0x00000000U)
#define CSL_I2C_SYSS_RDONE_RSTCOMP                              (0x00000001U)

#define CSL_I2C_SYSS_RESETVAL                                   (0x00000001U)

/* BUF */

#define CSL_I2C_BUF_TXTRSH_MASK                                 (0x0000003FU)
#define CSL_I2C_BUF_TXTRSH_SHIFT                                (0U)
#define CSL_I2C_BUF_TXTRSH_RESETVAL                             (0x00000000U)
#define CSL_I2C_BUF_TXTRSH_MAX                                  (0x0000003fU)

#define CSL_I2C_BUF_TXFIFO_CLR_MASK                             (0x00000040U)
#define CSL_I2C_BUF_TXFIFO_CLR_SHIFT                            (6U)
#define CSL_I2C_BUF_TXFIFO_CLR_RESETVAL                         (0x00000000U)
#define CSL_I2C_BUF_TXFIFO_CLR_NMODE                            (0x00000000U)
#define CSL_I2C_BUF_TXFIFO_CLR_RSTMODE                          (0x00000001U)

#define CSL_I2C_BUF_XDMA_EN_MASK                                (0x00000080U)
#define CSL_I2C_BUF_XDMA_EN_SHIFT                               (7U)
#define CSL_I2C_BUF_XDMA_EN_RESETVAL                            (0x00000000U)
#define CSL_I2C_BUF_XDMA_EN_DISABLE                             (0x00000000U)
#define CSL_I2C_BUF_XDMA_EN_ENABLE                              (0x00000001U)

#define CSL_I2C_BUF_RXTRSH_MASK                                 (0x00003F00U)
#define CSL_I2C_BUF_RXTRSH_SHIFT                                (8U)
#define CSL_I2C_BUF_RXTRSH_RESETVAL                             (0x00000000U)
#define CSL_I2C_BUF_RXTRSH_MAX                                  (0x0000003fU)

#define CSL_I2C_BUF_RXFIFO_CLR_MASK                             (0x00004000U)
#define CSL_I2C_BUF_RXFIFO_CLR_SHIFT                            (14U)
#define CSL_I2C_BUF_RXFIFO_CLR_RESETVAL                         (0x00000000U)
#define CSL_I2C_BUF_RXFIFO_CLR_NMODE                            (0x00000000U)
#define CSL_I2C_BUF_RXFIFO_CLR_RSTMODE                          (0x00000001U)

#define CSL_I2C_BUF_RDMA_EN_MASK                                (0x00008000U)
#define CSL_I2C_BUF_RDMA_EN_SHIFT                               (15U)
#define CSL_I2C_BUF_RDMA_EN_RESETVAL                            (0x00000000U)
#define CSL_I2C_BUF_RDMA_EN_DISABLE                             (0x00000000U)
#define CSL_I2C_BUF_RDMA_EN_ENABLE                              (0x00000001U)

#define CSL_I2C_BUF_RESETVAL                                    (0x00000000U)

/* CNT */

#define CSL_I2C_CNT_DCOUNT_MASK                                 (0x0000FFFFU)
#define CSL_I2C_CNT_DCOUNT_SHIFT                                (0U)
#define CSL_I2C_CNT_DCOUNT_RESETVAL                             (0x00000000U)
#define CSL_I2C_CNT_DCOUNT_MAX                                  (0x0000ffffU)

#define CSL_I2C_CNT_RESETVAL                                    (0x00000000U)

/* DATA */

#define CSL_I2C_DATA_DATA_MASK                                  (0x000000FFU)
#define CSL_I2C_DATA_DATA_SHIFT                                 (0U)
#define CSL_I2C_DATA_DATA_RESETVAL                              (0x00000000U)
#define CSL_I2C_DATA_DATA_MAX                                   (0x000000ffU)

#define CSL_I2C_DATA_RESETVAL                                   (0x00000000U)

/* CON */

#define CSL_I2C_CON_STT_MASK                                    (0x00000001U)
#define CSL_I2C_CON_STT_SHIFT                                   (0U)
#define CSL_I2C_CON_STT_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_STT_NSTT                                    (0x00000000U)
#define CSL_I2C_CON_STT_STT                                     (0x00000001U)

#define CSL_I2C_CON_STP_MASK                                    (0x00000002U)
#define CSL_I2C_CON_STP_SHIFT                                   (1U)
#define CSL_I2C_CON_STP_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_STP_NSTP                                    (0x00000000U)
#define CSL_I2C_CON_STP_STP                                     (0x00000001U)

#define CSL_I2C_CON_XOA3_MASK                                   (0x00000010U)
#define CSL_I2C_CON_XOA3_SHIFT                                  (4U)
#define CSL_I2C_CON_XOA3_RESETVAL                               (0x00000000U)
#define CSL_I2C_CON_XOA3_B07                                    (0x00000000U)
#define CSL_I2C_CON_XOA3_B10                                    (0x00000001U)

#define CSL_I2C_CON_XOA2_MASK                                   (0x00000020U)
#define CSL_I2C_CON_XOA2_SHIFT                                  (5U)
#define CSL_I2C_CON_XOA2_RESETVAL                               (0x00000000U)
#define CSL_I2C_CON_XOA2_B07                                    (0x00000000U)
#define CSL_I2C_CON_XOA2_B10                                    (0x00000001U)

#define CSL_I2C_CON_XOA1_MASK                                   (0x00000040U)
#define CSL_I2C_CON_XOA1_SHIFT                                  (6U)
#define CSL_I2C_CON_XOA1_RESETVAL                               (0x00000000U)
#define CSL_I2C_CON_XOA1_B07                                    (0x00000000U)
#define CSL_I2C_CON_XOA1_B10                                    (0x00000001U)

#define CSL_I2C_CON_XOA0_MASK                                   (0x00000080U)
#define CSL_I2C_CON_XOA0_SHIFT                                  (7U)
#define CSL_I2C_CON_XOA0_RESETVAL                               (0x00000000U)
#define CSL_I2C_CON_XOA0_B07                                    (0x00000000U)
#define CSL_I2C_CON_XOA0_B10                                    (0x00000001U)

#define CSL_I2C_CON_XSA_MASK                                    (0x00000100U)
#define CSL_I2C_CON_XSA_SHIFT                                   (8U)
#define CSL_I2C_CON_XSA_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_XSA_B07                                     (0x00000000U)
#define CSL_I2C_CON_XSA_B10                                     (0x00000001U)

#define CSL_I2C_CON_TRX_MASK                                    (0x00000200U)
#define CSL_I2C_CON_TRX_SHIFT                                   (9U)
#define CSL_I2C_CON_TRX_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_TRX_RCV                                     (0x00000000U)
#define CSL_I2C_CON_TRX_TRX                                     (0x00000001U)

#define CSL_I2C_CON_MST_MASK                                    (0x00000400U)
#define CSL_I2C_CON_MST_SHIFT                                   (10U)
#define CSL_I2C_CON_MST_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_MST_SLV                                     (0x00000000U)
#define CSL_I2C_CON_MST_MST                                     (0x00000001U)

#define CSL_I2C_CON_STB_MASK                                    (0x00000800U)
#define CSL_I2C_CON_STB_SHIFT                                   (11U)
#define CSL_I2C_CON_STB_RESETVAL                                (0x00000000U)
#define CSL_I2C_CON_STB_NORMAL                                  (0x00000000U)
#define CSL_I2C_CON_STB_STB                                     (0x00000001U)

#define CSL_I2C_CON_OPMODE_MASK                                 (0x00003000U)
#define CSL_I2C_CON_OPMODE_SHIFT                                (12U)
#define CSL_I2C_CON_OPMODE_RESETVAL                             (0x00000000U)
#define CSL_I2C_CON_OPMODE_FSI2C                                (0x00000000U)
#define CSL_I2C_CON_OPMODE_HSI2C                                (0x00000001U)
#define CSL_I2C_CON_OPMODE_SCCB                                 (0x00000002U)
#define CSL_I2C_CON_OPMODE_RESERVED                             (0x00000003U)

#define CSL_I2C_CON_I2C_EN_MASK                                 (0x00008000U)
#define CSL_I2C_CON_I2C_EN_SHIFT                                (15U)
#define CSL_I2C_CON_I2C_EN_RESETVAL                             (0x00000000U)
#define CSL_I2C_CON_I2C_EN_DISABLE                              (0x00000000U)
#define CSL_I2C_CON_I2C_EN_ENABLE                               (0x00000001U)

#define CSL_I2C_CON_RESETVAL                                    (0x00000000U)

/* OA */

#define CSL_I2C_OA_OA_MASK                                      (0x000003FFU)
#define CSL_I2C_OA_OA_SHIFT                                     (0U)
#define CSL_I2C_OA_OA_RESETVAL                                  (0x00000000U)
#define CSL_I2C_OA_OA_MAX                                       (0x000003ffU)

#define CSL_I2C_OA_MCODE_MASK                                   (0x0000E000U)
#define CSL_I2C_OA_MCODE_SHIFT                                  (13U)
#define CSL_I2C_OA_MCODE_RESETVAL                               (0x00000000U)
#define CSL_I2C_OA_MCODE_MAX                                    (0x00000007U)

#define CSL_I2C_OA_RESETVAL                                     (0x00000000U)

/* SA */

#define CSL_I2C_SA_SA_MASK                                      (0x000003FFU)
#define CSL_I2C_SA_SA_SHIFT                                     (0U)
#define CSL_I2C_SA_SA_RESETVAL                                  (0x000003ffU)
#define CSL_I2C_SA_SA_MAX                                       (0x000003ffU)

#define CSL_I2C_SA_RESETVAL                                     (0x000003ffU)

/* PSC */

#define CSL_I2C_PSC_PSC_MASK                                    (0x000000FFU)
#define CSL_I2C_PSC_PSC_SHIFT                                   (0U)
#define CSL_I2C_PSC_PSC_RESETVAL                                (0x00000000U)
#define CSL_I2C_PSC_PSC_MAX                                     (0x000000ffU)

#define CSL_I2C_PSC_RESETVAL                                    (0x00000000U)

/* SCLL */

#define CSL_I2C_SCLL_SCLL_MASK                                  (0x000000FFU)
#define CSL_I2C_SCLL_SCLL_SHIFT                                 (0U)
#define CSL_I2C_SCLL_SCLL_RESETVAL                              (0x00000000U)
#define CSL_I2C_SCLL_SCLL_MAX                                   (0x000000ffU)

#define CSL_I2C_SCLL_HSSCLL_MASK                                (0x0000FF00U)
#define CSL_I2C_SCLL_HSSCLL_SHIFT                               (8U)
#define CSL_I2C_SCLL_HSSCLL_RESETVAL                            (0x00000000U)
#define CSL_I2C_SCLL_HSSCLL_MAX                                 (0x000000ffU)

#define CSL_I2C_SCLL_RESETVAL                                   (0x00000000U)

/* SCLH */

#define CSL_I2C_SCLH_SCLH_MASK                                  (0x000000FFU)
#define CSL_I2C_SCLH_SCLH_SHIFT                                 (0U)
#define CSL_I2C_SCLH_SCLH_RESETVAL                              (0x00000000U)
#define CSL_I2C_SCLH_SCLH_MAX                                   (0x000000ffU)

#define CSL_I2C_SCLH_HSSCLH_MASK                                (0x0000FF00U)
#define CSL_I2C_SCLH_HSSCLH_SHIFT                               (8U)
#define CSL_I2C_SCLH_HSSCLH_RESETVAL                            (0x00000000U)
#define CSL_I2C_SCLH_HSSCLH_MAX                                 (0x000000ffU)

#define CSL_I2C_SCLH_RESETVAL                                   (0x00000000U)

/* SYSTEST */

#define CSL_I2C_SYSTEST_SDA_O_MASK                              (0x00000001U)
#define CSL_I2C_SYSTEST_SDA_O_SHIFT                             (0U)
#define CSL_I2C_SYSTEST_SDA_O_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_O_SDAOL                             (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_O_SDAOH                             (0x00000001U)

#define CSL_I2C_SYSTEST_SDA_I_MASK                              (0x00000002U)
#define CSL_I2C_SYSTEST_SDA_I_SHIFT                             (1U)
#define CSL_I2C_SYSTEST_SDA_I_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_I_SDAIL                             (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_I_SDAIH                             (0x00000001U)

#define CSL_I2C_SYSTEST_SCL_O_MASK                              (0x00000004U)
#define CSL_I2C_SYSTEST_SCL_O_SHIFT                             (2U)
#define CSL_I2C_SYSTEST_SCL_O_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_O_SCLOL                             (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_O_SCLOH                             (0x00000001U)

#define CSL_I2C_SYSTEST_SCCB_E_O_MASK                           (0x00000010U)
#define CSL_I2C_SYSTEST_SCCB_E_O_SHIFT                          (4U)
#define CSL_I2C_SYSTEST_SCCB_E_O_RESETVAL                       (0x00000000U)
#define CSL_I2C_SYSTEST_SCCB_E_O_SCCBOH                         (0x00000001U)
#define CSL_I2C_SYSTEST_SCCB_E_O_SCCBOL                         (0x00000000U)

#define CSL_I2C_SYSTEST_SDA_O_FUNC_MASK                         (0x00000020U)
#define CSL_I2C_SYSTEST_SDA_O_FUNC_SHIFT                        (5U)
#define CSL_I2C_SYSTEST_SDA_O_FUNC_RESETVAL                     (0x00000001U)
#define CSL_I2C_SYSTEST_SDA_O_FUNC_SDAOL                        (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_O_FUNC_SDAOH                        (0x00000001U)

#define CSL_I2C_SYSTEST_SDA_I_FUNC_MASK                         (0x00000040U)
#define CSL_I2C_SYSTEST_SDA_I_FUNC_SHIFT                        (6U)
#define CSL_I2C_SYSTEST_SDA_I_FUNC_RESETVAL                     (0x00000001U)
#define CSL_I2C_SYSTEST_SDA_I_FUNC_SDAIL                        (0x00000000U)
#define CSL_I2C_SYSTEST_SDA_I_FUNC_SDAIH                        (0x00000001U)

#define CSL_I2C_SYSTEST_SCL_O_FUNC_MASK                         (0x00000080U)
#define CSL_I2C_SYSTEST_SCL_O_FUNC_SHIFT                        (7U)
#define CSL_I2C_SYSTEST_SCL_O_FUNC_RESETVAL                     (0x00000001U)
#define CSL_I2C_SYSTEST_SCL_O_FUNC_SCLIL                        (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_O_FUNC_SCLIH                        (0x00000001U)

#define CSL_I2C_SYSTEST_SCL_I_FUNC_MASK                         (0x00000100U)
#define CSL_I2C_SYSTEST_SCL_I_FUNC_SHIFT                        (8U)
#define CSL_I2C_SYSTEST_SCL_I_FUNC_RESETVAL                     (0x00000001U)
#define CSL_I2C_SYSTEST_SCL_I_FUNC_SCLIL                        (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_I_FUNC_SCLIH                        (0x00000001U)

#define CSL_I2C_SYSTEST_SSB_MASK                                (0x00000800U)
#define CSL_I2C_SYSTEST_SSB_SHIFT                               (11U)
#define CSL_I2C_SYSTEST_SSB_RESETVAL                            (0x00000000U)
#define CSL_I2C_SYSTEST_SSB_NOACTION                            (0x00000000U)
#define CSL_I2C_SYSTEST_SSB_SETSTATUS                           (0x00000001U)

#define CSL_I2C_SYSTEST_TMODE_MASK                              (0x00003000U)
#define CSL_I2C_SYSTEST_TMODE_SHIFT                             (12U)
#define CSL_I2C_SYSTEST_TMODE_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_TMODE_FUNCTIONAL                        (0x00000000U)
#define CSL_I2C_SYSTEST_TMODE_RESERVED                          (0x00000001U)
#define CSL_I2C_SYSTEST_TMODE_TEST                              (0x00000002U)
#define CSL_I2C_SYSTEST_TMODE_LOOPBACK                          (0x00000003U)

#define CSL_I2C_SYSTEST_ST_EN_MASK                              (0x00008000U)
#define CSL_I2C_SYSTEST_ST_EN_SHIFT                             (15U)
#define CSL_I2C_SYSTEST_ST_EN_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_ST_EN_DISABLE                           (0x00000000U)
#define CSL_I2C_SYSTEST_ST_EN_ENABLE                            (0x00000001U)

#define CSL_I2C_SYSTEST_SCL_I_MASK                              (0x00000008U)
#define CSL_I2C_SYSTEST_SCL_I_SHIFT                             (3U)
#define CSL_I2C_SYSTEST_SCL_I_RESETVAL                          (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_I_SCLIL                             (0x00000000U)
#define CSL_I2C_SYSTEST_SCL_I_SCLIH                             (0x00000001U)

#define CSL_I2C_SYSTEST_FREE_MASK                               (0x00004000U)
#define CSL_I2C_SYSTEST_FREE_SHIFT                              (14U)
#define CSL_I2C_SYSTEST_FREE_RESETVAL                           (0x00000000U)
#define CSL_I2C_SYSTEST_FREE_STOP                               (0x00000000U)
#define CSL_I2C_SYSTEST_FREE_FREE                               (0x00000001U)

#define CSL_I2C_SYSTEST_RESETVAL                                (0x000001e0U)

/* BUFSTAT */

#define CSL_I2C_BUFSTAT_TXSTAT_MASK                             (0x0000003FU)
#define CSL_I2C_BUFSTAT_TXSTAT_SHIFT                            (0U)
#define CSL_I2C_BUFSTAT_TXSTAT_RESETVAL                         (0x00000000U)
#define CSL_I2C_BUFSTAT_TXSTAT_MAX                              (0x0000003fU)

#define CSL_I2C_BUFSTAT_RXSTAT_MASK                             (0x00003F00U)
#define CSL_I2C_BUFSTAT_RXSTAT_SHIFT                            (8U)
#define CSL_I2C_BUFSTAT_RXSTAT_RESETVAL                         (0x00000000U)
#define CSL_I2C_BUFSTAT_RXSTAT_MAX                              (0x0000003fU)

#define CSL_I2C_BUFSTAT_FIFODEPTH_MASK                          (0x0000C000U)
#define CSL_I2C_BUFSTAT_FIFODEPTH_SHIFT                         (14U)
#define CSL_I2C_BUFSTAT_FIFODEPTH_RESETVAL                      (0x00000003U)
#define CSL_I2C_BUFSTAT_FIFODEPTH_MAX                           (0x00000003U)

#define CSL_I2C_BUFSTAT_RESETVAL                                (0x0000c000U)

/* OA1 */

#define CSL_I2C_OA1_OA1_MASK                                    (0x000003FFU)
#define CSL_I2C_OA1_OA1_SHIFT                                   (0U)
#define CSL_I2C_OA1_OA1_RESETVAL                                (0x00000000U)
#define CSL_I2C_OA1_OA1_MAX                                     (0x000003ffU)

#define CSL_I2C_OA1_RESETVAL                                    (0x00000000U)

/* OA2 */

#define CSL_I2C_OA2_OA2_MASK                                    (0x000003FFU)
#define CSL_I2C_OA2_OA2_SHIFT                                   (0U)
#define CSL_I2C_OA2_OA2_RESETVAL                                (0x00000000U)
#define CSL_I2C_OA2_OA2_MAX                                     (0x000003ffU)

#define CSL_I2C_OA2_RESETVAL                                    (0x00000000U)

/* OA3 */

#define CSL_I2C_OA3_OA3_MASK                                    (0x000003FFU)
#define CSL_I2C_OA3_OA3_SHIFT                                   (0U)
#define CSL_I2C_OA3_OA3_RESETVAL                                (0x00000000U)
#define CSL_I2C_OA3_OA3_MAX                                     (0x000003ffU)

#define CSL_I2C_OA3_RESETVAL                                    (0x00000000U)

/* ACTOA */

#define CSL_I2C_ACTOA_OA0_ACT_MASK                              (0x00000001U)
#define CSL_I2C_ACTOA_OA0_ACT_SHIFT                             (0U)
#define CSL_I2C_ACTOA_OA0_ACT_RESETVAL                          (0x00000000U)
#define CSL_I2C_ACTOA_OA0_ACT_INACTIVE                          (0x00000000U)
#define CSL_I2C_ACTOA_OA0_ACT_ACTIVE                            (0x00000001U)

#define CSL_I2C_ACTOA_OA1_ACT_MASK                              (0x00000002U)
#define CSL_I2C_ACTOA_OA1_ACT_SHIFT                             (1U)
#define CSL_I2C_ACTOA_OA1_ACT_RESETVAL                          (0x00000000U)
#define CSL_I2C_ACTOA_OA1_ACT_ACTIVE                            (0x00000001U)
#define CSL_I2C_ACTOA_OA1_ACT_INACTIVE                          (0x00000000U)

#define CSL_I2C_ACTOA_OA2_ACT_MASK                              (0x00000004U)
#define CSL_I2C_ACTOA_OA2_ACT_SHIFT                             (2U)
#define CSL_I2C_ACTOA_OA2_ACT_RESETVAL                          (0x00000000U)
#define CSL_I2C_ACTOA_OA2_ACT_ACTIVE                            (0x00000001U)
#define CSL_I2C_ACTOA_OA2_ACT_INACTIVE                          (0x00000000U)

#define CSL_I2C_ACTOA_OA3_ACT_MASK                              (0x00000008U)
#define CSL_I2C_ACTOA_OA3_ACT_SHIFT                             (3U)
#define CSL_I2C_ACTOA_OA3_ACT_RESETVAL                          (0x00000000U)
#define CSL_I2C_ACTOA_OA3_ACT_ACTIVE                            (0x00000001U)
#define CSL_I2C_ACTOA_OA3_ACT_INACTIVE                          (0x00000000U)

#define CSL_I2C_ACTOA_RESETVAL                                  (0x00000000U)

/* SBLOCK */

#define CSL_I2C_SBLOCK_OA0_EN_MASK                              (0x00000001U)
#define CSL_I2C_SBLOCK_OA0_EN_SHIFT                             (0U)
#define CSL_I2C_SBLOCK_OA0_EN_RESETVAL                          (0x00000000U)
#define CSL_I2C_SBLOCK_OA0_EN_LOCK                              (0x00000001U)
#define CSL_I2C_SBLOCK_OA0_EN_UNLOCK                            (0x00000000U)

#define CSL_I2C_SBLOCK_OA1_EN_MASK                              (0x00000002U)
#define CSL_I2C_SBLOCK_OA1_EN_SHIFT                             (1U)
#define CSL_I2C_SBLOCK_OA1_EN_RESETVAL                          (0x00000000U)
#define CSL_I2C_SBLOCK_OA1_EN_LOCK                              (0x00000001U)
#define CSL_I2C_SBLOCK_OA1_EN_UNLOCK                            (0x00000000U)

#define CSL_I2C_SBLOCK_OA2_EN_MASK                              (0x00000004U)
#define CSL_I2C_SBLOCK_OA2_EN_SHIFT                             (2U)
#define CSL_I2C_SBLOCK_OA2_EN_RESETVAL                          (0x00000000U)
#define CSL_I2C_SBLOCK_OA2_EN_LOCK                              (0x00000001U)
#define CSL_I2C_SBLOCK_OA2_EN_UNLOCK                            (0x00000000U)

#define CSL_I2C_SBLOCK_OA3_EN_MASK                              (0x00000008U)
#define CSL_I2C_SBLOCK_OA3_EN_SHIFT                             (3U)
#define CSL_I2C_SBLOCK_OA3_EN_RESETVAL                          (0x00000000U)
#define CSL_I2C_SBLOCK_OA3_EN_LOCK                              (0x00000001U)
#define CSL_I2C_SBLOCK_OA3_EN_UNLOCK                            (0x00000000U)

#define CSL_I2C_SBLOCK_RESETVAL                                 (0x00000000U)

#define I2C_REVNB_LO_REVISION_SHIFT                                                                         ((uint32_t)0U)
#define I2C_REVNB_LO_REVISION_MASK                                                                          (0x0000ffffU)

#define I2C_REVNB_HI_REVISION_SHIFT                                                                         ((uint32_t)0U)
#define I2C_REVNB_HI_REVISION_MASK                                                                          (0x0000ffffU)

#define I2C_SYSC_AUTOIDLE_SHIFT                                                                             ((uint32_t)0U)
#define I2C_SYSC_AUTOIDLE_MASK                                                                              (0x00000001U)
#define I2C_SYSC_AUTOIDLE_DISABLE                                                                            ((uint32_t)0U)
#define I2C_SYSC_AUTOIDLE_ENABLE                                                                             ((uint32_t)1U)

#define I2C_SYSC_SRST_SHIFT                                                                                 ((uint32_t)1U)
#define I2C_SYSC_SRST_MASK                                                                                  (0x00000002U)
#define I2C_SYSC_SRST_NMODE                                                                                  ((uint32_t)0U)
#define I2C_SYSC_SRST_RSTMODE                                                                                ((uint32_t)1U)

#define I2C_SYSC_RESERVED_SHIFT                                                                             ((uint32_t)5U)
#define I2C_SYSC_RESERVED_MASK                                                                              (0x000000e0U)

#define I2C_SYSC_ENAWAKEUP_SHIFT                                                                            ((uint32_t)2U)
#define I2C_SYSC_ENAWAKEUP_MASK                                                                             (0x00000004U)
#define I2C_SYSC_ENAWAKEUP_DISABLE                                                                           ((uint32_t)0U)
#define I2C_SYSC_ENAWAKEUP_ENABLE                                                                            ((uint32_t)1U)

#define I2C_SYSC_RESERVED1_SHIFT                                                                            ((uint32_t)10U)
#define I2C_SYSC_RESERVED1_MASK                                                                             (0x0000fc00U)

#define I2C_SYSC_IDLEMODE_SHIFT                                                                             ((uint32_t)3U)
#define I2C_SYSC_IDLEMODE_MASK                                                                              (0x00000018U)
#define I2C_SYSC_IDLEMODE_FORCEIDLE                                                                          ((uint32_t)0U)
#define I2C_SYSC_IDLEMODE_SMARTIDLE_WAKEUP                                                                   ((uint32_t)3U)
#define I2C_SYSC_IDLEMODE_NOIDLE                                                                             ((uint32_t)1U)
#define I2C_SYSC_IDLEMODE_SMARTIDLE                                                                          ((uint32_t)2U)

#define I2C_SYSC_CLKACTIVITY_SHIFT                                                                          ((uint32_t)8U)
#define I2C_SYSC_CLKACTIVITY_MASK                                                                           (0x00000300U)
#define I2C_SYSC_CLKACTIVITY_BOOTHOFF                                                                        ((uint32_t)0U)
#define I2C_SYSC_CLKACTIVITY_OCPON                                                                           ((uint32_t)1U)
#define I2C_SYSC_CLKACTIVITY_SYSON                                                                           ((uint32_t)2U)
#define I2C_SYSC_CLKACTIVITY_BOOTHON                                                                         ((uint32_t)3U)

#define I2C_IRQSTATUS_RAW_AL_SHIFT                                                                          ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_AL_MASK                                                                           (0x00000001U)
#define I2C_IRQSTATUS_RAW_AL_CLEAR                                                                           ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_AL_SET                                                                             ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_NACK_SHIFT                                                                        ((uint32_t)1U)
#define I2C_IRQSTATUS_RAW_NACK_MASK                                                                         (0x00000002U)
#define I2C_IRQSTATUS_RAW_NACK_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_NACK_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_ARDY_SHIFT                                                                        ((uint32_t)2U)
#define I2C_IRQSTATUS_RAW_ARDY_MASK                                                                         (0x00000004U)
#define I2C_IRQSTATUS_RAW_ARDY_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_ARDY_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_RRDY_SHIFT                                                                        ((uint32_t)3U)
#define I2C_IRQSTATUS_RAW_RRDY_MASK                                                                         (0x00000008U)
#define I2C_IRQSTATUS_RAW_RRDY_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_RRDY_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_XRDY_SHIFT                                                                        ((uint32_t)4U)
#define I2C_IRQSTATUS_RAW_XRDY_MASK                                                                         (0x00000010U)
#define I2C_IRQSTATUS_RAW_XRDY_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_XRDY_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_GC_SHIFT                                                                          ((uint32_t)5U)
#define I2C_IRQSTATUS_RAW_GC_MASK                                                                           (0x00000020U)
#define I2C_IRQSTATUS_RAW_GC_CLEAR                                                                           ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_GC_SET                                                                             ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_STC_SHIFT                                                                         ((uint32_t)6U)
#define I2C_IRQSTATUS_RAW_STC_MASK                                                                          (0x00000040U)
#define I2C_IRQSTATUS_RAW_STC_CLEAR                                                                          ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_STC_SET                                                                            ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_AERR_SHIFT                                                                        ((uint32_t)7U)
#define I2C_IRQSTATUS_RAW_AERR_MASK                                                                         (0x00000080U)
#define I2C_IRQSTATUS_RAW_AERR_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_AERR_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_BF_SHIFT                                                                          ((uint32_t)8U)
#define I2C_IRQSTATUS_RAW_BF_MASK                                                                           (0x00000100U)
#define I2C_IRQSTATUS_RAW_BF_CLEAR                                                                           ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_BF_SET                                                                             ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_AAS_SHIFT                                                                         ((uint32_t)9U)
#define I2C_IRQSTATUS_RAW_AAS_MASK                                                                          (0x00000200U)
#define I2C_IRQSTATUS_RAW_AAS_CLEAR                                                                          ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_AAS_SET                                                                            ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_XUDF_SHIFT                                                                        ((uint32_t)10U)
#define I2C_IRQSTATUS_RAW_XUDF_MASK                                                                         (0x00000400U)
#define I2C_IRQSTATUS_RAW_XUDF_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_XUDF_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_ROVR_SHIFT                                                                        ((uint32_t)11U)
#define I2C_IRQSTATUS_RAW_ROVR_MASK                                                                         (0x00000800U)
#define I2C_IRQSTATUS_RAW_ROVR_CLEAR                                                                         ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_ROVR_SET                                                                           ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_BB_SHIFT                                                                          ((uint32_t)12U)
#define I2C_IRQSTATUS_RAW_BB_MASK                                                                           (0x00001000U)
#define I2C_IRQSTATUS_RAW_BB_CLEAR                                                                           ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_BB_SET                                                                             ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_RDR_SHIFT                                                                         ((uint32_t)13U)
#define I2C_IRQSTATUS_RAW_RDR_MASK                                                                          (0x00002000U)
#define I2C_IRQSTATUS_RAW_RDR_CLEAR                                                                          ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_RDR_SET                                                                            ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_XDR_SHIFT                                                                         ((uint32_t)14U)
#define I2C_IRQSTATUS_RAW_XDR_MASK                                                                          (0x00004000U)
#define I2C_IRQSTATUS_RAW_XDR_CLEAR                                                                          ((uint32_t)0U)
#define I2C_IRQSTATUS_RAW_XDR_SET                                                                            ((uint32_t)1U)

#define I2C_IRQSTATUS_RAW_RESERVED_11_SHIFT                                                                 ((uint32_t)15U)
#define I2C_IRQSTATUS_RAW_RESERVED_11_MASK                                                                  (0x00008000U)

#define I2C_IRQSTATUS_AL_SHIFT                                                                              ((uint32_t)0U)
#define I2C_IRQSTATUS_AL_MASK                                                                               (0x00000001U)
#define I2C_IRQSTATUS_AL_CLEAR                                                                               ((uint32_t)0U)
#define I2C_IRQSTATUS_AL_SET                                                                                 ((uint32_t)1U)

#define I2C_IRQSTATUS_NACK_SHIFT                                                                            ((uint32_t)1U)
#define I2C_IRQSTATUS_NACK_MASK                                                                             (0x00000002U)
#define I2C_IRQSTATUS_NACK_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_NACK_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_ARDY_SHIFT                                                                            ((uint32_t)2U)
#define I2C_IRQSTATUS_ARDY_MASK                                                                             (0x00000004U)
#define I2C_IRQSTATUS_ARDY_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_ARDY_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_RRDY_SHIFT                                                                            ((uint32_t)3U)
#define I2C_IRQSTATUS_RRDY_MASK                                                                             (0x00000008U)
#define I2C_IRQSTATUS_RRDY_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_RRDY_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_XRDY_SHIFT                                                                            ((uint32_t)4U)
#define I2C_IRQSTATUS_XRDY_MASK                                                                             (0x00000010U)
#define I2C_IRQSTATUS_XRDY_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_XRDY_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_GC_SHIFT                                                                              ((uint32_t)5U)
#define I2C_IRQSTATUS_GC_MASK                                                                               (0x00000020U)
#define I2C_IRQSTATUS_GC_CLEAR                                                                               ((uint32_t)0U)
#define I2C_IRQSTATUS_GC_SET                                                                                 ((uint32_t)1U)

#define I2C_IRQSTATUS_STC_SHIFT                                                                             ((uint32_t)6U)
#define I2C_IRQSTATUS_STC_MASK                                                                              (0x00000040U)
#define I2C_IRQSTATUS_STC_CLEAR                                                                              ((uint32_t)0U)
#define I2C_IRQSTATUS_STC_SET                                                                                ((uint32_t)1U)

#define I2C_IRQSTATUS_AERR_SHIFT                                                                            ((uint32_t)7U)
#define I2C_IRQSTATUS_AERR_MASK                                                                             (0x00000080U)
#define I2C_IRQSTATUS_AERR_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_AERR_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_BF_SHIFT                                                                              ((uint32_t)8U)
#define I2C_IRQSTATUS_BF_MASK                                                                               (0x00000100U)
#define I2C_IRQSTATUS_BF_CLEAR                                                                               ((uint32_t)0U)
#define I2C_IRQSTATUS_BF_SET                                                                                 ((uint32_t)1U)

#define I2C_IRQSTATUS_AAS_SHIFT                                                                             ((uint32_t)9U)
#define I2C_IRQSTATUS_AAS_MASK                                                                              (0x00000200U)
#define I2C_IRQSTATUS_AAS_CLEAR                                                                              ((uint32_t)0U)
#define I2C_IRQSTATUS_AAS_SET                                                                                ((uint32_t)1U)

#define I2C_IRQSTATUS_XUDF_SHIFT                                                                            ((uint32_t)10U)
#define I2C_IRQSTATUS_XUDF_MASK                                                                             (0x00000400U)
#define I2C_IRQSTATUS_XUDF_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_XUDF_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_ROVR_SHIFT                                                                            ((uint32_t)11U)
#define I2C_IRQSTATUS_ROVR_MASK                                                                             (0x00000800U)
#define I2C_IRQSTATUS_ROVR_CLEAR                                                                             ((uint32_t)0U)
#define I2C_IRQSTATUS_ROVR_SET                                                                               ((uint32_t)1U)

#define I2C_IRQSTATUS_BB_SHIFT                                                                              ((uint32_t)12U)
#define I2C_IRQSTATUS_BB_MASK                                                                               (0x00001000U)
#define I2C_IRQSTATUS_BB_CLEAR                                                                               ((uint32_t)0U)
#define I2C_IRQSTATUS_BB_SET                                                                                 ((uint32_t)1U)

#define I2C_IRQSTATUS_RDR_SHIFT                                                                             ((uint32_t)13U)
#define I2C_IRQSTATUS_RDR_MASK                                                                              (0x00002000U)
#define I2C_IRQSTATUS_RDR_CLEAR                                                                              ((uint32_t)0U)
#define I2C_IRQSTATUS_RDR_SET                                                                                ((uint32_t)1U)

#define I2C_IRQSTATUS_XDR_SHIFT                                                                             ((uint32_t)14U)
#define I2C_IRQSTATUS_XDR_MASK                                                                              (0x00004000U)
#define I2C_IRQSTATUS_XDR_CLEAR                                                                              ((uint32_t)0U)
#define I2C_IRQSTATUS_XDR_SET                                                                                ((uint32_t)1U)

#define I2C_IRQSTATUS_RESERVED_11_SHIFT                                                                     ((uint32_t)15U)
#define I2C_IRQSTATUS_RESERVED_11_MASK                                                                      (0x00008000U)

#define I2C_IRQENABLE_SET_AL_IE_SHIFT                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_SET_AL_IE_MASK                                                                        (0x00000001U)
#define I2C_IRQENABLE_SET_AL_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_SET_AL_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_SET_NACK_IE_SHIFT                                                                     ((uint32_t)1U)
#define I2C_IRQENABLE_SET_NACK_IE_MASK                                                                      (0x00000002U)
#define I2C_IRQENABLE_SET_NACK_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_SET_NACK_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_SET_ARDY_IE_SHIFT                                                                     ((uint32_t)2U)
#define I2C_IRQENABLE_SET_ARDY_IE_MASK                                                                      (0x00000004U)
#define I2C_IRQENABLE_SET_ARDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_SET_ARDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_SET_RRDY_IE_SHIFT                                                                     ((uint32_t)3U)
#define I2C_IRQENABLE_SET_RRDY_IE_MASK                                                                      (0x00000008U)
#define I2C_IRQENABLE_SET_RRDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_SET_RRDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_SET_XRDY_IE_SHIFT                                                                     ((uint32_t)4U)
#define I2C_IRQENABLE_SET_XRDY_IE_MASK                                                                      (0x00000010U)
#define I2C_IRQENABLE_SET_XRDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_SET_XRDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_SET_GC_IE_SHIFT                                                                       ((uint32_t)5U)
#define I2C_IRQENABLE_SET_GC_IE_MASK                                                                        (0x00000020U)
#define I2C_IRQENABLE_SET_GC_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_SET_GC_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_SET_STC_IE_SHIFT                                                                      ((uint32_t)6U)
#define I2C_IRQENABLE_SET_STC_IE_MASK                                                                       (0x00000040U)
#define I2C_IRQENABLE_SET_STC_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_SET_STC_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_SET_AERR_IE_SHIFT                                                                     ((uint32_t)7U)
#define I2C_IRQENABLE_SET_AERR_IE_MASK                                                                      (0x00000080U)
#define I2C_IRQENABLE_SET_AERR_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_SET_AERR_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_SET_BF_IE_SHIFT                                                                       ((uint32_t)8U)
#define I2C_IRQENABLE_SET_BF_IE_MASK                                                                        (0x00000100U)
#define I2C_IRQENABLE_SET_BF_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_SET_BF_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_SET_ASS_IE_SHIFT                                                                      ((uint32_t)9U)
#define I2C_IRQENABLE_SET_ASS_IE_MASK                                                                       (0x00000200U)
#define I2C_IRQENABLE_SET_ASS_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_SET_ASS_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_SET_XUDF_SHIFT                                                                        ((uint32_t)10U)
#define I2C_IRQENABLE_SET_XUDF_MASK                                                                         (0x00000400U)
#define I2C_IRQENABLE_SET_XUDF_DISABLE                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_SET_XUDF_ENABLE                                                                        ((uint32_t)1U)

#define I2C_IRQENABLE_SET_ROVR_SHIFT                                                                        ((uint32_t)11U)
#define I2C_IRQENABLE_SET_ROVR_MASK                                                                         (0x00000800U)
#define I2C_IRQENABLE_SET_ROVR_DISABLE                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_SET_ROVR_ENABLE                                                                        ((uint32_t)1U)

#define I2C_IRQENABLE_SET_RESERVED1_SHIFT                                                                   ((uint32_t)12U)
#define I2C_IRQENABLE_SET_RESERVED1_MASK                                                                    (0x00001000U)

#define I2C_IRQENABLE_SET_RDR_IE_SHIFT                                                                      ((uint32_t)13U)
#define I2C_IRQENABLE_SET_RDR_IE_MASK                                                                       (0x00002000U)
#define I2C_IRQENABLE_SET_RDR_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_SET_RDR_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_SET_XDR_IE_SHIFT                                                                      ((uint32_t)14U)
#define I2C_IRQENABLE_SET_XDR_IE_MASK                                                                       (0x00004000U)
#define I2C_IRQENABLE_SET_XDR_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_SET_XDR_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_SET_RESERVED_5_SHIFT                                                                  ((uint32_t)15U)
#define I2C_IRQENABLE_SET_RESERVED_5_MASK                                                                   (0x00008000U)

#define I2C_IRQENABLE_CLR_AL_IE_SHIFT                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_AL_IE_MASK                                                                        (0x00000001U)
#define I2C_IRQENABLE_CLR_AL_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_AL_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_NACK_IE_SHIFT                                                                     ((uint32_t)1U)
#define I2C_IRQENABLE_CLR_NACK_IE_MASK                                                                      (0x00000002U)
#define I2C_IRQENABLE_CLR_NACK_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_NACK_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_ARDY_IE_SHIFT                                                                     ((uint32_t)2U)
#define I2C_IRQENABLE_CLR_ARDY_IE_MASK                                                                      (0x00000004U)
#define I2C_IRQENABLE_CLR_ARDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_ARDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_RRDY_IE_SHIFT                                                                     ((uint32_t)3U)
#define I2C_IRQENABLE_CLR_RRDY_IE_MASK                                                                      (0x00000008U)
#define I2C_IRQENABLE_CLR_RRDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_RRDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_XRDY_IE_SHIFT                                                                     ((uint32_t)4U)
#define I2C_IRQENABLE_CLR_XRDY_IE_MASK                                                                      (0x00000010U)
#define I2C_IRQENABLE_CLR_XRDY_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_XRDY_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_GC_IE_SHIFT                                                                       ((uint32_t)5U)
#define I2C_IRQENABLE_CLR_GC_IE_MASK                                                                        (0x00000020U)
#define I2C_IRQENABLE_CLR_GC_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_GC_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_STC_IE_SHIFT                                                                      ((uint32_t)6U)
#define I2C_IRQENABLE_CLR_STC_IE_MASK                                                                       (0x00000040U)
#define I2C_IRQENABLE_CLR_STC_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_STC_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_AERR_IE_SHIFT                                                                     ((uint32_t)7U)
#define I2C_IRQENABLE_CLR_AERR_IE_MASK                                                                      (0x00000080U)
#define I2C_IRQENABLE_CLR_AERR_IE_DISABLE                                                                    ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_AERR_IE_ENABLE                                                                     ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_BF_IE_SHIFT                                                                       ((uint32_t)8U)
#define I2C_IRQENABLE_CLR_BF_IE_MASK                                                                        (0x00000100U)
#define I2C_IRQENABLE_CLR_BF_IE_DISABLE                                                                      ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_BF_IE_ENABLE                                                                       ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_ASS_IE_SHIFT                                                                      ((uint32_t)9U)
#define I2C_IRQENABLE_CLR_ASS_IE_MASK                                                                       (0x00000200U)
#define I2C_IRQENABLE_CLR_ASS_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_ASS_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_XUDF_SHIFT                                                                        ((uint32_t)10U)
#define I2C_IRQENABLE_CLR_XUDF_MASK                                                                         (0x00000400U)
#define I2C_IRQENABLE_CLR_XUDF_DISABLE                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_XUDF_ENABLE                                                                        ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_ROVR_SHIFT                                                                        ((uint32_t)11U)
#define I2C_IRQENABLE_CLR_ROVR_MASK                                                                         (0x00000800U)
#define I2C_IRQENABLE_CLR_ROVR_DISABLE                                                                       ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_ROVR_ENABLE                                                                        ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_RESERVED1_SHIFT                                                                   ((uint32_t)12U)
#define I2C_IRQENABLE_CLR_RESERVED1_MASK                                                                    (0x00001000U)

#define I2C_IRQENABLE_CLR_RDR_IE_SHIFT                                                                      ((uint32_t)13U)
#define I2C_IRQENABLE_CLR_RDR_IE_MASK                                                                       (0x00002000U)
#define I2C_IRQENABLE_CLR_RDR_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_RDR_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_XDR_IE_SHIFT                                                                      ((uint32_t)14U)
#define I2C_IRQENABLE_CLR_XDR_IE_MASK                                                                       (0x00004000U)
#define I2C_IRQENABLE_CLR_XDR_IE_DISABLE                                                                     ((uint32_t)0U)
#define I2C_IRQENABLE_CLR_XDR_IE_ENABLE                                                                      ((uint32_t)1U)

#define I2C_IRQENABLE_CLR_RESERVED_5_SHIFT                                                                  ((uint32_t)15U)
#define I2C_IRQENABLE_CLR_RESERVED_5_MASK                                                                   (0x00008000U)

#define I2C_WE_AL_SHIFT                                                                                     ((uint32_t)0U)
#define I2C_WE_AL_MASK                                                                                      (0x00000001U)
#define I2C_WE_AL_ENABLE                                                                                     ((uint32_t)1U)
#define I2C_WE_AL_DISABLE                                                                                    ((uint32_t)0U)

#define I2C_WE_NACK_SHIFT                                                                                   ((uint32_t)1U)
#define I2C_WE_NACK_MASK                                                                                    (0x00000002U)
#define I2C_WE_NACK_DISABLE                                                                                  ((uint32_t)0U)
#define I2C_WE_NACK_ENABLE                                                                                   ((uint32_t)1U)

#define I2C_WE_ARDY_SHIFT                                                                                   ((uint32_t)2U)
#define I2C_WE_ARDY_MASK                                                                                    (0x00000004U)
#define I2C_WE_ARDY_ENABLE                                                                                   ((uint32_t)1U)
#define I2C_WE_ARDY_DISABLE                                                                                  ((uint32_t)0U)

#define I2C_WE_DRDY_SHIFT                                                                                   ((uint32_t)3U)
#define I2C_WE_DRDY_MASK                                                                                    (0x00000008U)
#define I2C_WE_DRDY_DISABLE                                                                                  ((uint32_t)0U)
#define I2C_WE_DRDY_ENABLE                                                                                   ((uint32_t)1U)

#define I2C_WE_RESERVED1_SHIFT                                                                              ((uint32_t)4U)
#define I2C_WE_RESERVED1_MASK                                                                               (0x00000010U)

#define I2C_WE_GC_SHIFT                                                                                     ((uint32_t)5U)
#define I2C_WE_GC_MASK                                                                                      (0x00000020U)
#define I2C_WE_GC_DISABLE                                                                                    ((uint32_t)0U)
#define I2C_WE_GC_ENABLE                                                                                     ((uint32_t)1U)

#define I2C_WE_STC_SHIFT                                                                                    ((uint32_t)6U)
#define I2C_WE_STC_MASK                                                                                     (0x00000040U)
#define I2C_WE_STC_DISABLE                                                                                   ((uint32_t)0U)
#define I2C_WE_STC_ENABLE                                                                                    ((uint32_t)1U)

#define I2C_WE_RESERVED2_SHIFT                                                                              ((uint32_t)7U)
#define I2C_WE_RESERVED2_MASK                                                                               (0x00000080U)

#define I2C_WE_BF_SHIFT                                                                                     ((uint32_t)8U)
#define I2C_WE_BF_MASK                                                                                      (0x00000100U)
#define I2C_WE_BF_DISABLE                                                                                    ((uint32_t)0U)
#define I2C_WE_BF_ENABLE                                                                                     ((uint32_t)1U)

#define I2C_WE_AAS_SHIFT                                                                                    ((uint32_t)9U)
#define I2C_WE_AAS_MASK                                                                                     (0x00000200U)
#define I2C_WE_AAS_DISABLE                                                                                   ((uint32_t)0U)
#define I2C_WE_AAS_ENABLE                                                                                    ((uint32_t)1U)

#define I2C_WE_XUDF_SHIFT                                                                                   ((uint32_t)10U)
#define I2C_WE_XUDF_MASK                                                                                    (0x00000400U)
#define I2C_WE_XUDF_DISABLE                                                                                  ((uint32_t)0U)
#define I2C_WE_XUDF_ENABLE                                                                                   ((uint32_t)1U)

#define I2C_WE_ROVR_SHIFT                                                                                   ((uint32_t)11U)
#define I2C_WE_ROVR_MASK                                                                                    (0x00000800U)
#define I2C_WE_ROVR_DISABLE                                                                                  ((uint32_t)0U)
#define I2C_WE_ROVR_ENABLE                                                                                   ((uint32_t)1U)

#define I2C_WE_RESERVED3_SHIFT                                                                              ((uint32_t)12U)
#define I2C_WE_RESERVED3_MASK                                                                               (0x00001000U)

#define I2C_WE_RDR_SHIFT                                                                                    ((uint32_t)13U)
#define I2C_WE_RDR_MASK                                                                                     (0x00002000U)
#define I2C_WE_RDR_DISABLE                                                                                   ((uint32_t)0U)
#define I2C_WE_RDR_ENABLE                                                                                    ((uint32_t)1U)

#define I2C_WE_XDR_SHIFT                                                                                    ((uint32_t)14U)
#define I2C_WE_XDR_MASK                                                                                     (0x00004000U)
#define I2C_WE_XDR_DISABLE                                                                                   ((uint32_t)0U)
#define I2C_WE_XDR_ENABLE                                                                                    ((uint32_t)1U)

#define I2C_WE_RESERVED_SHIFT                                                                               ((uint32_t)15U)
#define I2C_WE_RESERVED_MASK                                                                                (0x00008000U)

#define I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_SHIFT                                                          ((uint32_t)0U)
#define I2C_DMARXENABLE_SET_DMARX_ENABLE_SET_MASK                                                           (0x00000001U)

#define I2C_DMARXENABLE_SET_RESERVED_SHIFT                                                                  ((uint32_t)1U)
#define I2C_DMARXENABLE_SET_RESERVED_MASK                                                                   (0x0000fffeU)

#define I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_SHIFT                                                          ((uint32_t)0U)
#define I2C_DMATXENABLE_SET_DMATX_ENABLE_SET_MASK                                                           (0x00000001U)

#define I2C_DMATXENABLE_SET_RESERVED_SHIFT                                                                  ((uint32_t)1U)
#define I2C_DMATXENABLE_SET_RESERVED_MASK                                                                   (0x0000fffeU)

#define I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_SHIFT                                                        ((uint32_t)0U)
#define I2C_DMARXENABLE_CLR_DMARX_ENABLE_CLEAR_MASK                                                         (0x00000001U)

#define I2C_DMARXENABLE_CLR_RESERVED_SHIFT                                                                  ((uint32_t)1U)
#define I2C_DMARXENABLE_CLR_RESERVED_MASK                                                                   (0x0000fffeU)

#define I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_SHIFT                                                        ((uint32_t)0U)
#define I2C_DMATXENABLE_CLR_DMATX_ENABLE_CLEAR_MASK                                                         (0x00000001U)

#define I2C_DMATXENABLE_CLR_RESERVED_SHIFT                                                                  ((uint32_t)1U)
#define I2C_DMATXENABLE_CLR_RESERVED_MASK                                                                   (0x0000fffeU)

#define I2C_DMARXWAKE_EN_AL_SHIFT                                                                           ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_AL_MASK                                                                            (0x00000001U)
#define I2C_DMARXWAKE_EN_AL_ENABLE                                                                           ((uint32_t)1U)
#define I2C_DMARXWAKE_EN_AL_DISABLE                                                                          ((uint32_t)0U)

#define I2C_DMARXWAKE_EN_NACK_SHIFT                                                                         ((uint32_t)1U)
#define I2C_DMARXWAKE_EN_NACK_MASK                                                                          (0x00000002U)
#define I2C_DMARXWAKE_EN_NACK_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_NACK_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_ARDY_SHIFT                                                                         ((uint32_t)2U)
#define I2C_DMARXWAKE_EN_ARDY_MASK                                                                          (0x00000004U)
#define I2C_DMARXWAKE_EN_ARDY_ENABLE                                                                         ((uint32_t)1U)
#define I2C_DMARXWAKE_EN_ARDY_DISABLE                                                                        ((uint32_t)0U)

#define I2C_DMARXWAKE_EN_DRDY_SHIFT                                                                         ((uint32_t)3U)
#define I2C_DMARXWAKE_EN_DRDY_MASK                                                                          (0x00000008U)
#define I2C_DMARXWAKE_EN_DRDY_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_DRDY_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_RESERVED1_SHIFT                                                                    ((uint32_t)4U)
#define I2C_DMARXWAKE_EN_RESERVED1_MASK                                                                     (0x00000010U)

#define I2C_DMARXWAKE_EN_GC_SHIFT                                                                           ((uint32_t)5U)
#define I2C_DMARXWAKE_EN_GC_MASK                                                                            (0x00000020U)
#define I2C_DMARXWAKE_EN_GC_DISABLE                                                                          ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_GC_ENABLE                                                                           ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_STC_SHIFT                                                                          ((uint32_t)6U)
#define I2C_DMARXWAKE_EN_STC_MASK                                                                           (0x00000040U)
#define I2C_DMARXWAKE_EN_STC_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_STC_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_RESERVED2_SHIFT                                                                    ((uint32_t)7U)
#define I2C_DMARXWAKE_EN_RESERVED2_MASK                                                                     (0x00000080U)

#define I2C_DMARXWAKE_EN_BF_SHIFT                                                                           ((uint32_t)8U)
#define I2C_DMARXWAKE_EN_BF_MASK                                                                            (0x00000100U)
#define I2C_DMARXWAKE_EN_BF_DISABLE                                                                          ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_BF_ENABLE                                                                           ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_AAS_SHIFT                                                                          ((uint32_t)9U)
#define I2C_DMARXWAKE_EN_AAS_MASK                                                                           (0x00000200U)
#define I2C_DMARXWAKE_EN_AAS_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_AAS_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_XUDF_SHIFT                                                                         ((uint32_t)10U)
#define I2C_DMARXWAKE_EN_XUDF_MASK                                                                          (0x00000400U)
#define I2C_DMARXWAKE_EN_XUDF_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_XUDF_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_ROVR_SHIFT                                                                         ((uint32_t)11U)
#define I2C_DMARXWAKE_EN_ROVR_MASK                                                                          (0x00000800U)
#define I2C_DMARXWAKE_EN_ROVR_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_ROVR_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_RESERVED3_SHIFT                                                                    ((uint32_t)12U)
#define I2C_DMARXWAKE_EN_RESERVED3_MASK                                                                     (0x00001000U)

#define I2C_DMARXWAKE_EN_RDR_SHIFT                                                                          ((uint32_t)13U)
#define I2C_DMARXWAKE_EN_RDR_MASK                                                                           (0x00002000U)
#define I2C_DMARXWAKE_EN_RDR_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_RDR_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_XDR_SHIFT                                                                          ((uint32_t)14U)
#define I2C_DMARXWAKE_EN_XDR_MASK                                                                           (0x00004000U)
#define I2C_DMARXWAKE_EN_XDR_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMARXWAKE_EN_XDR_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMARXWAKE_EN_RESERVED_SHIFT                                                                     ((uint32_t)15U)
#define I2C_DMARXWAKE_EN_RESERVED_MASK                                                                      (0x00008000U)

#define I2C_DMATXWAKE_EN_AL_SHIFT                                                                           ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_AL_MASK                                                                            (0x00000001U)
#define I2C_DMATXWAKE_EN_AL_ENABLE                                                                           ((uint32_t)1U)
#define I2C_DMATXWAKE_EN_AL_DISABLE                                                                          ((uint32_t)0U)

#define I2C_DMATXWAKE_EN_NACK_SHIFT                                                                         ((uint32_t)1U)
#define I2C_DMATXWAKE_EN_NACK_MASK                                                                          (0x00000002U)
#define I2C_DMATXWAKE_EN_NACK_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_NACK_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_ARDY_SHIFT                                                                         ((uint32_t)2U)
#define I2C_DMATXWAKE_EN_ARDY_MASK                                                                          (0x00000004U)
#define I2C_DMATXWAKE_EN_ARDY_ENABLE                                                                         ((uint32_t)1U)
#define I2C_DMATXWAKE_EN_ARDY_DISABLE                                                                        ((uint32_t)0U)

#define I2C_DMATXWAKE_EN_DRDY_SHIFT                                                                         ((uint32_t)3U)
#define I2C_DMATXWAKE_EN_DRDY_MASK                                                                          (0x00000008U)
#define I2C_DMATXWAKE_EN_DRDY_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_DRDY_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_RESERVED1_SHIFT                                                                    ((uint32_t)4U)
#define I2C_DMATXWAKE_EN_RESERVED1_MASK                                                                     (0x00000010U)

#define I2C_DMATXWAKE_EN_GC_SHIFT                                                                           ((uint32_t)5U)
#define I2C_DMATXWAKE_EN_GC_MASK                                                                            (0x00000020U)
#define I2C_DMATXWAKE_EN_GC_DISABLE                                                                          ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_GC_ENABLE                                                                           ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_STC_SHIFT                                                                          ((uint32_t)6U)
#define I2C_DMATXWAKE_EN_STC_MASK                                                                           (0x00000040U)
#define I2C_DMATXWAKE_EN_STC_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_STC_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_RESERVED2_SHIFT                                                                    ((uint32_t)7U)
#define I2C_DMATXWAKE_EN_RESERVED2_MASK                                                                     (0x00000080U)

#define I2C_DMATXWAKE_EN_BF_SHIFT                                                                           ((uint32_t)8U)
#define I2C_DMATXWAKE_EN_BF_MASK                                                                            (0x00000100U)
#define I2C_DMATXWAKE_EN_BF_DISABLE                                                                          ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_BF_ENABLE                                                                           ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_AAS_SHIFT                                                                          ((uint32_t)9U)
#define I2C_DMATXWAKE_EN_AAS_MASK                                                                           (0x00000200U)
#define I2C_DMATXWAKE_EN_AAS_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_AAS_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_XUDF_SHIFT                                                                         ((uint32_t)10U)
#define I2C_DMATXWAKE_EN_XUDF_MASK                                                                          (0x00000400U)
#define I2C_DMATXWAKE_EN_XUDF_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_XUDF_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_ROVR_SHIFT                                                                         ((uint32_t)11U)
#define I2C_DMATXWAKE_EN_ROVR_MASK                                                                          (0x00000800U)
#define I2C_DMATXWAKE_EN_ROVR_DISABLE                                                                        ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_ROVR_ENABLE                                                                         ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_RESERVED3_SHIFT                                                                    ((uint32_t)12U)
#define I2C_DMATXWAKE_EN_RESERVED3_MASK                                                                     (0x00001000U)

#define I2C_DMATXWAKE_EN_RDR_SHIFT                                                                          ((uint32_t)13U)
#define I2C_DMATXWAKE_EN_RDR_MASK                                                                           (0x00002000U)
#define I2C_DMATXWAKE_EN_RDR_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_RDR_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_XDR_SHIFT                                                                          ((uint32_t)14U)
#define I2C_DMATXWAKE_EN_XDR_MASK                                                                           (0x00004000U)
#define I2C_DMATXWAKE_EN_XDR_DISABLE                                                                         ((uint32_t)0U)
#define I2C_DMATXWAKE_EN_XDR_ENABLE                                                                          ((uint32_t)1U)

#define I2C_DMATXWAKE_EN_RESERVED_SHIFT                                                                     ((uint32_t)15U)
#define I2C_DMATXWAKE_EN_RESERVED_MASK                                                                      (0x00008000U)

#define I2C_SYSS_RDONE_SHIFT                                                                                ((uint32_t)0U)
#define I2C_SYSS_RDONE_MASK                                                                                 (0x00000001U)
#define I2C_SYSS_RDONE_RSTONGOING                                                                            ((uint32_t)0U)
#define I2C_SYSS_RDONE_RSTCOMP                                                                               ((uint32_t)1U)

#define I2C_SYSS_RESERVED_SHIFT                                                                             ((uint32_t)1U)
#define I2C_SYSS_RESERVED_MASK                                                                              (0x0000fffeU)

#define I2C_BUF_TXTRSH_SHIFT                                                                                ((uint32_t)0U)
#define I2C_BUF_TXTRSH_MASK                                                                                 (0x0000003fU)

#define I2C_BUF_TXFIFO_CLR_SHIFT                                                                            ((uint32_t)6U)
#define I2C_BUF_TXFIFO_CLR_MASK                                                                             (0x00000040U)
#define I2C_BUF_TXFIFO_CLR_NMODE                                                                             ((uint32_t)0U)
#define I2C_BUF_TXFIFO_CLR_RSTMODE                                                                           ((uint32_t)1U)

#define I2C_BUF_XDMA_EN_SHIFT                                                                               ((uint32_t)7U)
#define I2C_BUF_XDMA_EN_MASK                                                                                (0x00000080U)
#define I2C_BUF_XDMA_EN_DISABLE                                                                              ((uint32_t)0U)
#define I2C_BUF_XDMA_EN_ENABLE                                                                               ((uint32_t)1U)

#define I2C_BUF_RXTRSH_SHIFT                                                                                ((uint32_t)8U)
#define I2C_BUF_RXTRSH_MASK                                                                                 (0x00003f00U)

#define I2C_BUF_RXFIFO_CLR_SHIFT                                                                            ((uint32_t)14U)
#define I2C_BUF_RXFIFO_CLR_MASK                                                                             (0x00004000U)
#define I2C_BUF_RXFIFO_CLR_NMODE                                                                             ((uint32_t)0U)
#define I2C_BUF_RXFIFO_CLR_RSTMODE                                                                           ((uint32_t)1U)

#define I2C_BUF_RDMA_EN_SHIFT                                                                               ((uint32_t)15U)
#define I2C_BUF_RDMA_EN_MASK                                                                                (0x00008000U)
#define I2C_BUF_RDMA_EN_DISABLE                                                                              ((uint32_t)0U)
#define I2C_BUF_RDMA_EN_ENABLE                                                                               ((uint32_t)1U)

#define I2C_CNT_DCOUNT_SHIFT                                                                                ((uint32_t)0U)
#define I2C_CNT_DCOUNT_MASK                                                                                 (0x0000ffffU)

#define I2C_DATA_DATA_SHIFT                                                                                 ((uint32_t)0U)
#define I2C_DATA_DATA_MASK                                                                                  (0x000000ffU)

#define I2C_DATA_RESERVED_SHIFT                                                                             ((uint32_t)8U)
#define I2C_DATA_RESERVED_MASK                                                                              (0x0000ff00U)

#define I2C_CON_STT_SHIFT                                                                                   ((uint32_t)0U)
#define I2C_CON_STT_MASK                                                                                    (0x00000001U)
#define I2C_CON_STT_NSTT                                                                                     ((uint32_t)0U)
#define I2C_CON_STT_STT                                                                                      ((uint32_t)1U)

#define I2C_CON_STP_SHIFT                                                                                   ((uint32_t)1U)
#define I2C_CON_STP_MASK                                                                                    (0x00000002U)
#define I2C_CON_STP_NSTP                                                                                     ((uint32_t)0U)
#define I2C_CON_STP_STP                                                                                      ((uint32_t)1U)

#define I2C_CON_RESERVED1_SHIFT                                                                             ((uint32_t)2U)
#define I2C_CON_RESERVED1_MASK                                                                              (0x0000000cU)

#define I2C_CON_XOA3_SHIFT                                                                                  ((uint32_t)4U)
#define I2C_CON_XOA3_MASK                                                                                   (0x00000010U)
#define I2C_CON_XOA3_B07                                                                                     ((uint32_t)0U)
#define I2C_CON_XOA3_B10                                                                                     ((uint32_t)1U)

#define I2C_CON_XOA2_SHIFT                                                                                  ((uint32_t)5U)
#define I2C_CON_XOA2_MASK                                                                                   (0x00000020U)
#define I2C_CON_XOA2_B07                                                                                     ((uint32_t)0U)
#define I2C_CON_XOA2_B10                                                                                     ((uint32_t)1U)

#define I2C_CON_XOA1_SHIFT                                                                                  ((uint32_t)6U)
#define I2C_CON_XOA1_MASK                                                                                   (0x00000040U)
#define I2C_CON_XOA1_B07                                                                                     ((uint32_t)0U)
#define I2C_CON_XOA1_B10                                                                                     ((uint32_t)1U)

#define I2C_CON_XOA0_SHIFT                                                                                  ((uint32_t)7U)
#define I2C_CON_XOA0_MASK                                                                                   (0x00000080U)
#define I2C_CON_XOA0_B07                                                                                     ((uint32_t)0U)
#define I2C_CON_XOA0_B10                                                                                     ((uint32_t)1U)

#define I2C_CON_XSA_SHIFT                                                                                   ((uint32_t)8U)
#define I2C_CON_XSA_MASK                                                                                    (0x00000100U)
#define I2C_CON_XSA_B07                                                                                      ((uint32_t)0U)
#define I2C_CON_XSA_B10                                                                                      ((uint32_t)1U)

#define I2C_CON_TRX_SHIFT                                                                                   ((uint32_t)9U)
#define I2C_CON_TRX_MASK                                                                                    (0x00000200U)
#define I2C_CON_TRX_RCV                                                                                      ((uint32_t)0U)
#define I2C_CON_TRX_TRX                                                                                      ((uint32_t)1U)

#define I2C_CON_MST_SHIFT                                                                                   ((uint32_t)10U)
#define I2C_CON_MST_MASK                                                                                    (0x00000400U)
#define I2C_CON_MST_SLV                                                                                      ((uint32_t)0U)
#define I2C_CON_MST_MST                                                                                      ((uint32_t)1U)

#define I2C_CON_STB_SHIFT                                                                                   ((uint32_t)11U)
#define I2C_CON_STB_MASK                                                                                    (0x00000800U)
#define I2C_CON_STB_NORMAL                                                                                   ((uint32_t)0U)
#define I2C_CON_STB_STB                                                                                      ((uint32_t)1U)

#define I2C_CON_OPMODE_SHIFT                                                                                ((uint32_t)12U)
#define I2C_CON_OPMODE_MASK                                                                                 (0x00003000U)
#define I2C_CON_OPMODE_FSI2C                                                                                 ((uint32_t)0U)
#define I2C_CON_OPMODE_HSI2C                                                                                 ((uint32_t)1U)
#define I2C_CON_OPMODE_SCCB                                                                                  ((uint32_t)2U)
#define I2C_CON_OPMODE_RESERVED                                                                              ((uint32_t)3U)

#define I2C_CON_RESERVED_SHIFT                                                                              ((uint32_t)14U)
#define I2C_CON_RESERVED_MASK                                                                               (0x00004000U)

#define I2C_CON_I2C_EN_SHIFT                                                                                ((uint32_t)15U)
#define I2C_CON_I2C_EN_MASK                                                                                 (0x00008000U)
#define I2C_CON_I2C_EN_DISABLE                                                                               ((uint32_t)0U)
#define I2C_CON_I2C_EN_ENABLE                                                                                ((uint32_t)1U)

#define I2C_OA_OA_SHIFT                                                                                     ((uint32_t)0U)
#define I2C_OA_OA_MASK                                                                                      (0x000003ffU)

#define I2C_OA_RESERVED_SHIFT                                                                               ((uint32_t)10U)
#define I2C_OA_RESERVED_MASK                                                                                (0x00001c00U)

#define I2C_OA_MCODE_SHIFT                                                                                  ((uint32_t)13U)
#define I2C_OA_MCODE_MASK                                                                                   (0x0000e000U)

#define I2C_SA_SA_SHIFT                                                                                     ((uint32_t)0U)
#define I2C_SA_SA_MASK                                                                                      (0x000003ffU)

#define I2C_SA_RESERVED_SHIFT                                                                               ((uint32_t)10U)
#define I2C_SA_RESERVED_MASK                                                                                (0x0000fc00U)

#define I2C_PSC_PSC_SHIFT                                                                                   ((uint32_t)0U)
#define I2C_PSC_PSC_MASK                                                                                    (0x000000ffU)

#define I2C_PSC_RESERVED_SHIFT                                                                              ((uint32_t)8U)
#define I2C_PSC_RESERVED_MASK                                                                               (0x0000ff00U)

#define I2C_SCLL_SCLL_SHIFT                                                                                 ((uint32_t)0U)
#define I2C_SCLL_SCLL_MASK                                                                                  (0x000000ffU)

#define I2C_SCLL_HSSCLL_SHIFT                                                                               ((uint32_t)8U)
#define I2C_SCLL_HSSCLL_MASK                                                                                (0x0000ff00U)

#define I2C_SCLH_SCLH_SHIFT                                                                                 ((uint32_t)0U)
#define I2C_SCLH_SCLH_MASK                                                                                  (0x000000ffU)

#define I2C_SCLH_HSSCLH_SHIFT                                                                               ((uint32_t)8U)
#define I2C_SCLH_HSSCLH_MASK                                                                                (0x0000ff00U)

#define I2C_SYSTEST_SDA_O_SHIFT                                                                             ((uint32_t)0U)
#define I2C_SYSTEST_SDA_O_MASK                                                                              (0x00000001U)
#define I2C_SYSTEST_SDA_O_SDAOL                                                                              ((uint32_t)0U)
#define I2C_SYSTEST_SDA_O_SDAOH                                                                              ((uint32_t)1U)

#define I2C_SYSTEST_SDA_I_SHIFT                                                                             ((uint32_t)1U)
#define I2C_SYSTEST_SDA_I_MASK                                                                              (0x00000002U)
#define I2C_SYSTEST_SDA_I_SDAIL                                                                              ((uint32_t)0U)
#define I2C_SYSTEST_SDA_I_SDAIH                                                                              ((uint32_t)1U)

#define I2C_SYSTEST_SCL_O_SHIFT                                                                             ((uint32_t)2U)
#define I2C_SYSTEST_SCL_O_MASK                                                                              (0x00000004U)
#define I2C_SYSTEST_SCL_O_SCLOL                                                                              ((uint32_t)0U)
#define I2C_SYSTEST_SCL_O_SCLOH                                                                              ((uint32_t)1U)

#define I2C_SYSTEST_SDA_O_FUNC_SHIFT                                                                        ((uint32_t)5U)
#define I2C_SYSTEST_SDA_O_FUNC_MASK                                                                         (0x00000020U)
#define I2C_SYSTEST_SDA_O_FUNC_SDAOL                                                                         ((uint32_t)0U)
#define I2C_SYSTEST_SDA_O_FUNC_SDAOH                                                                         ((uint32_t)1U)

#define I2C_SYSTEST_SDA_I_FUNC_SHIFT                                                                        ((uint32_t)6U)
#define I2C_SYSTEST_SDA_I_FUNC_MASK                                                                         (0x00000040U)
#define I2C_SYSTEST_SDA_I_FUNC_SDAIL                                                                         ((uint32_t)0U)
#define I2C_SYSTEST_SDA_I_FUNC_SDAIH                                                                         ((uint32_t)1U)

#define I2C_SYSTEST_SCL_O_FUNC_SHIFT                                                                        ((uint32_t)7U)
#define I2C_SYSTEST_SCL_O_FUNC_MASK                                                                         (0x00000080U)
#define I2C_SYSTEST_SCL_O_FUNC_SCLIL                                                                         ((uint32_t)0U)
#define I2C_SYSTEST_SCL_O_FUNC_SCLIH                                                                         ((uint32_t)1U)

#define I2C_SYSTEST_SCL_I_FUNC_SHIFT                                                                        ((uint32_t)8U)
#define I2C_SYSTEST_SCL_I_FUNC_MASK                                                                         (0x00000100U)
#define I2C_SYSTEST_SCL_I_FUNC_SCLIL                                                                         ((uint32_t)0U)
#define I2C_SYSTEST_SCL_I_FUNC_SCLIH                                                                         ((uint32_t)1U)

#define I2C_SYSTEST_SSB_SHIFT                                                                               ((uint32_t)11U)
#define I2C_SYSTEST_SSB_MASK                                                                                (0x00000800U)
#define I2C_SYSTEST_SSB_NOACTION                                                                             ((uint32_t)0U)
#define I2C_SYSTEST_SSB_SETSTATUS                                                                            ((uint32_t)1U)

#define I2C_SYSTEST_TMODE_SHIFT                                                                             ((uint32_t)12U)
#define I2C_SYSTEST_TMODE_MASK                                                                              (0x00003000U)
#define I2C_SYSTEST_TMODE_FUNCTIONAL                                                                         ((uint32_t)0U)
#define I2C_SYSTEST_TMODE_RESERVED                                                                           ((uint32_t)1U)
#define I2C_SYSTEST_TMODE_TEST                                                                               ((uint32_t)2U)
#define I2C_SYSTEST_TMODE_LOOPBACK                                                                           ((uint32_t)3U)

#define I2C_SYSTEST_ST_EN_SHIFT                                                                             ((uint32_t)15U)
#define I2C_SYSTEST_ST_EN_MASK                                                                              (0x00008000U)
#define I2C_SYSTEST_ST_EN_DISABLE                                                                            ((uint32_t)0U)
#define I2C_SYSTEST_ST_EN_ENABLE                                                                             ((uint32_t)1U)

#define I2C_SYSTEST_SCL_I_SHIFT                                                                             ((uint32_t)3U)
#define I2C_SYSTEST_SCL_I_MASK                                                                              (0x00000008U)
#define I2C_SYSTEST_SCL_I_SCLIL                                                                              ((uint32_t)0U)
#define I2C_SYSTEST_SCL_I_SCLIH                                                                              ((uint32_t)1U)

#define I2C_SYSTEST_FREE_SHIFT                                                                              ((uint32_t)14U)
#define I2C_SYSTEST_FREE_MASK                                                                               (0x00004000U)
#define I2C_SYSTEST_FREE_STOP                                                                                ((uint32_t)0U)
#define I2C_SYSTEST_FREE_FREE                                                                                ((uint32_t)1U)

#define I2C_SYSTEST_RESERVED_1_SHIFT                                                                        ((uint32_t)4U)
#define I2C_SYSTEST_RESERVED_1_MASK                                                                         (0x00000010U)
#define I2C_SYSTEST_RESERVED_1_SCCBOH                                                                        ((uint32_t)1U)
#define I2C_SYSTEST_RESERVED_1_SCCBOL                                                                        ((uint32_t)0U)

#define I2C_SYSTEST_RESERVED_2_SHIFT                                                                        ((uint32_t)9U)
#define I2C_SYSTEST_RESERVED_2_MASK                                                                         (0x00000600U)

#define I2C_BUFSTAT_TXSTAT_SHIFT                                                                            ((uint32_t)0U)
#define I2C_BUFSTAT_TXSTAT_MASK                                                                             (0x0000003fU)

#define I2C_BUFSTAT_RXSTAT_SHIFT                                                                            ((uint32_t)8U)
#define I2C_BUFSTAT_RXSTAT_MASK                                                                             (0x00003f00U)

#define I2C_BUFSTAT_RESERVED_SHIFT                                                                          ((uint32_t)6U)
#define I2C_BUFSTAT_RESERVED_MASK                                                                           (0x000000c0U)

#define I2C_BUFSTAT_FIFODEPTH_SHIFT                                                                         ((uint32_t)14U)
#define I2C_BUFSTAT_FIFODEPTH_MASK                                                                          (0x0000c000U)

#define I2C_OA1_OA1_SHIFT                                                                                   ((uint32_t)0U)
#define I2C_OA1_OA1_MASK                                                                                    (0x000003ffU)

#define I2C_OA1_RESERVED_SHIFT                                                                              ((uint32_t)10U)
#define I2C_OA1_RESERVED_MASK                                                                               (0x0000fc00U)

#define I2C_OA2_OA2_SHIFT                                                                                   ((uint32_t)0U)
#define I2C_OA2_OA2_MASK                                                                                    (0x000003ffU)

#define I2C_OA2_RESERVED_SHIFT                                                                              ((uint32_t)10U)
#define I2C_OA2_RESERVED_MASK                                                                               (0x0000fc00U)

#define I2C_OA3_OA3_SHIFT                                                                                   ((uint32_t)0U)
#define I2C_OA3_OA3_MASK                                                                                    (0x000003ffU)

#define I2C_OA3_RESERVED_SHIFT                                                                              ((uint32_t)10U)
#define I2C_OA3_RESERVED_MASK                                                                               (0x0000fc00U)

#define I2C_ACTOA_OA0_ACT_SHIFT                                                                             ((uint32_t)0U)
#define I2C_ACTOA_OA0_ACT_MASK                                                                              (0x00000001U)
#define I2C_ACTOA_OA0_ACT_INACTIVE                                                                           ((uint32_t)0U)
#define I2C_ACTOA_OA0_ACT_ACTIVE                                                                             ((uint32_t)1U)

#define I2C_ACTOA_RESERVED_SHIFT                                                                            ((uint32_t)4U)
#define I2C_ACTOA_RESERVED_MASK                                                                             (0x0000fff0U)

#define I2C_ACTOA_OA1_ACT_SHIFT                                                                             ((uint32_t)1U)
#define I2C_ACTOA_OA1_ACT_MASK                                                                              (0x00000002U)
#define I2C_ACTOA_OA1_ACT_ACTIVE                                                                             ((uint32_t)1U)
#define I2C_ACTOA_OA1_ACT_INACTIVE                                                                           ((uint32_t)0U)

#define I2C_ACTOA_OA2_ACT_SHIFT                                                                             ((uint32_t)2U)
#define I2C_ACTOA_OA2_ACT_MASK                                                                              (0x00000004U)
#define I2C_ACTOA_OA2_ACT_ACTIVE                                                                             ((uint32_t)1U)
#define I2C_ACTOA_OA2_ACT_INACTIVE                                                                           ((uint32_t)0U)

#define I2C_ACTOA_OA3_ACT_SHIFT                                                                             ((uint32_t)3U)
#define I2C_ACTOA_OA3_ACT_MASK                                                                              (0x00000008U)
#define I2C_ACTOA_OA3_ACT_ACTIVE                                                                             ((uint32_t)1U)
#define I2C_ACTOA_OA3_ACT_INACTIVE                                                                           ((uint32_t)0U)

#define I2C_SBLOCK_OA0_EN_SHIFT                                                                             ((uint32_t)0U)
#define I2C_SBLOCK_OA0_EN_MASK                                                                              (0x00000001U)
#define I2C_SBLOCK_OA0_EN_LOCK                                                                               ((uint32_t)1U)
#define I2C_SBLOCK_OA0_EN_UNLOCK                                                                             ((uint32_t)0U)

#define I2C_SBLOCK_RESERVED_SHIFT                                                                           ((uint32_t)4U)
#define I2C_SBLOCK_RESERVED_MASK                                                                            (0x0000fff0U)

#define I2C_SBLOCK_OA1_EN_SHIFT                                                                             ((uint32_t)1U)
#define I2C_SBLOCK_OA1_EN_MASK                                                                              (0x00000002U)
#define I2C_SBLOCK_OA1_EN_LOCK                                                                               ((uint32_t)1U)
#define I2C_SBLOCK_OA1_EN_UNLOCK                                                                             ((uint32_t)0U)

#define I2C_SBLOCK_OA2_EN_SHIFT                                                                             ((uint32_t)2U)
#define I2C_SBLOCK_OA2_EN_MASK                                                                              (0x00000004U)
#define I2C_SBLOCK_OA2_EN_LOCK                                                                               ((uint32_t)1U)
#define I2C_SBLOCK_OA2_EN_UNLOCK                                                                             ((uint32_t)0U)

#define I2C_SBLOCK_OA3_EN_SHIFT                                                                             ((uint32_t)3U)
#define I2C_SBLOCK_OA3_EN_MASK                                                                              (0x00000008U)
#define I2C_SBLOCK_OA3_EN_LOCK                                                                               ((uint32_t)1U)
#define I2C_SBLOCK_OA3_EN_UNLOCK                                                                             ((uint32_t)0U)

#ifdef __cplusplus
}
#endif
#endif  /* _HW_I2C_H_ */
