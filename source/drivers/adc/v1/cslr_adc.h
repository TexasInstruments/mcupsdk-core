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

#ifndef CSLR_ADC_H_
#define CSLR_ADC_H_

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
    volatile uint16_t ADCCTL1;
    volatile uint16_t ADCCTL2;
    volatile uint16_t ADCBURSTCTL;
    volatile uint16_t ADCINTFLG;
    volatile uint16_t ADCINTFLGCLR;
    volatile uint16_t ADCINTOVF;
    volatile uint16_t ADCINTOVFCLR;
    volatile uint16_t ADCINTSEL1N2;
    volatile uint16_t ADCINTSEL3N4;
    volatile uint16_t ADCSOCPRICTL;
    volatile uint16_t ADCINTSOCSEL1;
    volatile uint16_t ADCINTSOCSEL2;
    volatile uint16_t ADCSOCFLG1;
    volatile uint16_t ADCSOCFRC1;
    volatile uint16_t ADCSOCOVF1;
    volatile uint16_t ADCSOCOVFCLR1;
    volatile uint32_t ADCSOC0CTL;
    volatile uint32_t ADCSOC1CTL;
    volatile uint32_t ADCSOC2CTL;
    volatile uint32_t ADCSOC3CTL;
    volatile uint32_t ADCSOC4CTL;
    volatile uint32_t ADCSOC5CTL;
    volatile uint32_t ADCSOC6CTL;
    volatile uint32_t ADCSOC7CTL;
    volatile uint32_t ADCSOC8CTL;
    volatile uint32_t ADCSOC9CTL;
    volatile uint32_t ADCSOC10CTL;
    volatile uint32_t ADCSOC11CTL;
    volatile uint32_t ADCSOC12CTL;
    volatile uint32_t ADCSOC13CTL;
    volatile uint32_t ADCSOC14CTL;
    volatile uint32_t ADCSOC15CTL;
    volatile uint16_t ADCEVTSTAT;
    volatile uint8_t  Resv_100[2];
    volatile uint16_t ADCEVTCLR;
    volatile uint8_t  Resv_104[2];
    volatile uint16_t ADCEVTSEL;
    volatile uint8_t  Resv_108[2];
    volatile uint16_t ADCEVTINTSEL;
    volatile uint8_t  Resv_112[2];
    volatile uint16_t ADCOSDETECT;
    volatile uint16_t ADCCOUNTER;
    volatile uint16_t ADCREV;
    volatile uint16_t ADCOFFTRIM;
    volatile uint8_t  Resv_124[4];
    volatile uint32_t ADCCONFIG;
    volatile uint16_t ADCPPB1CONFIG;
    volatile uint16_t ADCPPB1STAMP;
    volatile uint16_t ADCPPB1OFFCAL;
    volatile uint16_t ADCPPB1OFFREF;
    volatile uint32_t ADCPPB1TRIPHI;
    volatile uint32_t ADCPPB1TRIPLO;
    volatile uint16_t ADCPPB2CONFIG;
    volatile uint16_t ADCPPB2STAMP;
    volatile uint16_t ADCPPB2OFFCAL;
    volatile uint16_t ADCPPB2OFFREF;
    volatile uint32_t ADCPPB2TRIPHI;
    volatile uint32_t ADCPPB2TRIPLO;
    volatile uint16_t ADCPPB3CONFIG;
    volatile uint16_t ADCPPB3STAMP;
    volatile uint16_t ADCPPB3OFFCAL;
    volatile uint16_t ADCPPB3OFFREF;
    volatile uint32_t ADCPPB3TRIPHI;
    volatile uint32_t ADCPPB3TRIPLO;
    volatile uint16_t ADCPPB4CONFIG;
    volatile uint16_t ADCPPB4STAMP;
    volatile uint16_t ADCPPB4OFFCAL;
    volatile uint16_t ADCPPB4OFFREF;
    volatile uint32_t ADCPPB4TRIPHI;
    volatile uint32_t ADCPPB4TRIPLO;
    volatile uint8_t  Resv_222[30];
    volatile uint16_t ADCINTCYCLE;
    volatile uint32_t ADCINLTRIM1;
    volatile uint32_t ADCINLTRIM2;
    volatile uint32_t ADCINLTRIM3;
    volatile uint32_t ADCINLTRIM4;
    volatile uint32_t ADCINLTRIM5;
    volatile uint32_t ADCINLTRIM6;
    volatile uint8_t  Resv_252[4];
    volatile uint32_t ADCINLTRIMCTL;
} CSL_adcRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ADC_ADCCTL1                                                        (0x00000000U)
#define CSL_ADC_ADCCTL2                                                        (0x00000002U)
#define CSL_ADC_ADCBURSTCTL                                                    (0x00000004U)
#define CSL_ADC_ADCINTFLG                                                      (0x00000006U)
#define CSL_ADC_ADCINTFLGCLR                                                   (0x00000008U)
#define CSL_ADC_ADCINTOVF                                                      (0x0000000AU)
#define CSL_ADC_ADCINTOVFCLR                                                   (0x0000000CU)
#define CSL_ADC_ADCINTSEL1N2                                                   (0x0000000EU)
#define CSL_ADC_ADCINTSEL3N4                                                   (0x00000010U)
#define CSL_ADC_ADCSOCPRICTL                                                   (0x00000012U)
#define CSL_ADC_ADCINTSOCSEL1                                                  (0x00000014U)
#define CSL_ADC_ADCINTSOCSEL2                                                  (0x00000016U)
#define CSL_ADC_ADCSOCFLG1                                                     (0x00000018U)
#define CSL_ADC_ADCSOCFRC1                                                     (0x0000001AU)
#define CSL_ADC_ADCSOCOVF1                                                     (0x0000001CU)
#define CSL_ADC_ADCSOCOVFCLR1                                                  (0x0000001EU)
#define CSL_ADC_ADCSOC0CTL                                                     (0x00000020U)
#define CSL_ADC_ADCSOC1CTL                                                     (0x00000024U)
#define CSL_ADC_ADCSOC2CTL                                                     (0x00000028U)
#define CSL_ADC_ADCSOC3CTL                                                     (0x0000002CU)
#define CSL_ADC_ADCSOC4CTL                                                     (0x00000030U)
#define CSL_ADC_ADCSOC5CTL                                                     (0x00000034U)
#define CSL_ADC_ADCSOC6CTL                                                     (0x00000038U)
#define CSL_ADC_ADCSOC7CTL                                                     (0x0000003CU)
#define CSL_ADC_ADCSOC8CTL                                                     (0x00000040U)
#define CSL_ADC_ADCSOC9CTL                                                     (0x00000044U)
#define CSL_ADC_ADCSOC10CTL                                                    (0x00000048U)
#define CSL_ADC_ADCSOC11CTL                                                    (0x0000004CU)
#define CSL_ADC_ADCSOC12CTL                                                    (0x00000050U)
#define CSL_ADC_ADCSOC13CTL                                                    (0x00000054U)
#define CSL_ADC_ADCSOC14CTL                                                    (0x00000058U)
#define CSL_ADC_ADCSOC15CTL                                                    (0x0000005CU)
#define CSL_ADC_ADCEVTSTAT                                                     (0x00000060U)
#define CSL_ADC_ADCEVTCLR                                                      (0x00000064U)
#define CSL_ADC_ADCEVTSEL                                                      (0x00000068U)
#define CSL_ADC_ADCEVTINTSEL                                                   (0x0000006CU)
#define CSL_ADC_ADCOSDETECT                                                    (0x00000070U)
#define CSL_ADC_ADCCOUNTER                                                     (0x00000072U)
#define CSL_ADC_ADCREV                                                         (0x00000074U)
#define CSL_ADC_ADCOFFTRIM                                                     (0x00000076U)
#define CSL_ADC_ADCCONFIG                                                      (0x0000007CU)
#define CSL_ADC_ADCPPB1CONFIG                                                  (0x00000080U)
#define CSL_ADC_ADCPPB1STAMP                                                   (0x00000082U)
#define CSL_ADC_ADCPPB1OFFCAL                                                  (0x00000084U)
#define CSL_ADC_ADCPPB1OFFREF                                                  (0x00000086U)
#define CSL_ADC_ADCPPB1TRIPHI                                                  (0x00000088U)
#define CSL_ADC_ADCPPB1TRIPLO                                                  (0x0000008CU)
#define CSL_ADC_ADCPPB2CONFIG                                                  (0x00000090U)
#define CSL_ADC_ADCPPB2STAMP                                                   (0x00000092U)
#define CSL_ADC_ADCPPB2OFFCAL                                                  (0x00000094U)
#define CSL_ADC_ADCPPB2OFFREF                                                  (0x00000096U)
#define CSL_ADC_ADCPPB2TRIPHI                                                  (0x00000098U)
#define CSL_ADC_ADCPPB2TRIPLO                                                  (0x0000009CU)
#define CSL_ADC_ADCPPB3CONFIG                                                  (0x000000A0U)
#define CSL_ADC_ADCPPB3STAMP                                                   (0x000000A2U)
#define CSL_ADC_ADCPPB3OFFCAL                                                  (0x000000A4U)
#define CSL_ADC_ADCPPB3OFFREF                                                  (0x000000A6U)
#define CSL_ADC_ADCPPB3TRIPHI                                                  (0x000000A8U)
#define CSL_ADC_ADCPPB3TRIPLO                                                  (0x000000ACU)
#define CSL_ADC_ADCPPB4CONFIG                                                  (0x000000B0U)
#define CSL_ADC_ADCPPB4STAMP                                                   (0x000000B2U)
#define CSL_ADC_ADCPPB4OFFCAL                                                  (0x000000B4U)
#define CSL_ADC_ADCPPB4OFFREF                                                  (0x000000B6U)
#define CSL_ADC_ADCPPB4TRIPHI                                                  (0x000000B8U)
#define CSL_ADC_ADCPPB4TRIPLO                                                  (0x000000BCU)
#define CSL_ADC_ADCINTCYCLE                                                    (0x000000DEU)
#define CSL_ADC_ADCINLTRIM1                                                    (0x000000E0U)
#define CSL_ADC_ADCINLTRIM2                                                    (0x000000E4U)
#define CSL_ADC_ADCINLTRIM3                                                    (0x000000E8U)
#define CSL_ADC_ADCINLTRIM4                                                    (0x000000ECU)
#define CSL_ADC_ADCINLTRIM5                                                    (0x000000F0U)
#define CSL_ADC_ADCINLTRIM6                                                    (0x000000F4U)
#define CSL_ADC_ADCINLTRIMCTL                                                  (0x000000FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ADCCTL1 */

#define CSL_ADC_ADCCTL1_RESERVED_1_MASK                                        (0x0003U)
#define CSL_ADC_ADCCTL1_RESERVED_1_SHIFT                                       (0x0000U)
#define CSL_ADC_ADCCTL1_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL1_RESERVED_1_MAX                                         (0x0003U)

#define CSL_ADC_ADCCTL1_INTPULSEPOS_MASK                                       (0x0004U)
#define CSL_ADC_ADCCTL1_INTPULSEPOS_SHIFT                                      (0x0002U)
#define CSL_ADC_ADCCTL1_INTPULSEPOS_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCCTL1_INTPULSEPOS_MAX                                        (0x0001U)

#define CSL_ADC_ADCCTL1_RESERVED_2_MASK                                        (0x0078U)
#define CSL_ADC_ADCCTL1_RESERVED_2_SHIFT                                       (0x0003U)
#define CSL_ADC_ADCCTL1_RESERVED_2_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL1_RESERVED_2_MAX                                         (0x000FU)

#define CSL_ADC_ADCCTL1_ADCPWDNZ_MASK                                          (0x0080U)
#define CSL_ADC_ADCCTL1_ADCPWDNZ_SHIFT                                         (0x0007U)
#define CSL_ADC_ADCCTL1_ADCPWDNZ_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCCTL1_ADCPWDNZ_MAX                                           (0x0001U)

#define CSL_ADC_ADCCTL1_ADCBSYCHN_MASK                                         (0x0F00U)
#define CSL_ADC_ADCCTL1_ADCBSYCHN_SHIFT                                        (0x0008U)
#define CSL_ADC_ADCCTL1_ADCBSYCHN_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCCTL1_ADCBSYCHN_MAX                                          (0x000FU)

#define CSL_ADC_ADCCTL1_RESERVED_3_MASK                                        (0x1000U)
#define CSL_ADC_ADCCTL1_RESERVED_3_SHIFT                                       (0x000CU)
#define CSL_ADC_ADCCTL1_RESERVED_3_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL1_RESERVED_3_MAX                                         (0x0001U)

#define CSL_ADC_ADCCTL1_ADCBSY_MASK                                            (0x2000U)
#define CSL_ADC_ADCCTL1_ADCBSY_SHIFT                                           (0x000DU)
#define CSL_ADC_ADCCTL1_ADCBSY_RESETVAL                                        (0x0000U)
#define CSL_ADC_ADCCTL1_ADCBSY_MAX                                             (0x0001U)

#define CSL_ADC_ADCCTL1_RESERVED_4_MASK                                        (0xC000U)
#define CSL_ADC_ADCCTL1_RESERVED_4_SHIFT                                       (0x000EU)
#define CSL_ADC_ADCCTL1_RESERVED_4_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL1_RESERVED_4_MAX                                         (0x0003U)

#define CSL_ADC_ADCCTL1_RESETVAL                                               (0x0000U)

/* ADCCTL2 */

#define CSL_ADC_ADCCTL2_PRESCALE_MASK                                          (0x000FU)
#define CSL_ADC_ADCCTL2_PRESCALE_SHIFT                                         (0x0000U)
#define CSL_ADC_ADCCTL2_PRESCALE_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCCTL2_PRESCALE_MAX                                           (0x000FU)

#define CSL_ADC_ADCCTL2_RESERVED_1_MASK                                        (0x0030U)
#define CSL_ADC_ADCCTL2_RESERVED_1_SHIFT                                       (0x0004U)
#define CSL_ADC_ADCCTL2_RESERVED_1_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL2_RESERVED_1_MAX                                         (0x0003U)

#define CSL_ADC_ADCCTL2_RESOLUTION_MASK                                        (0x0040U)
#define CSL_ADC_ADCCTL2_RESOLUTION_SHIFT                                       (0x0006U)
#define CSL_ADC_ADCCTL2_RESOLUTION_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL2_RESOLUTION_MAX                                         (0x0001U)

#define CSL_ADC_ADCCTL2_SIGNALMODE_MASK                                        (0x0080U)
#define CSL_ADC_ADCCTL2_SIGNALMODE_SHIFT                                       (0x0007U)
#define CSL_ADC_ADCCTL2_SIGNALMODE_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL2_SIGNALMODE_MAX                                         (0x0001U)

#define CSL_ADC_ADCCTL2_RESERVED_2_MASK                                        (0x1F00U)
#define CSL_ADC_ADCCTL2_RESERVED_2_SHIFT                                       (0x0008U)
#define CSL_ADC_ADCCTL2_RESERVED_2_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL2_RESERVED_2_MAX                                         (0x001FU)

#define CSL_ADC_ADCCTL2_RESERVED_3_MASK                                        (0xE000U)
#define CSL_ADC_ADCCTL2_RESERVED_3_SHIFT                                       (0x000DU)
#define CSL_ADC_ADCCTL2_RESERVED_3_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCCTL2_RESERVED_3_MAX                                         (0x0007U)

#define CSL_ADC_ADCCTL2_RESETVAL                                               (0x0000U)

/* ADCBURSTCTL */

#define CSL_ADC_ADCBURSTCTL_BURSTTRIGSEL_MASK                                  (0x007FU)
#define CSL_ADC_ADCBURSTCTL_BURSTTRIGSEL_SHIFT                                 (0x0000U)
#define CSL_ADC_ADCBURSTCTL_BURSTTRIGSEL_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCBURSTCTL_BURSTTRIGSEL_MAX                                   (0x007FU)

#define CSL_ADC_ADCBURSTCTL_RESERVED_1_MASK                                    (0x0080U)
#define CSL_ADC_ADCBURSTCTL_RESERVED_1_SHIFT                                   (0x0007U)
#define CSL_ADC_ADCBURSTCTL_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_ADC_ADCBURSTCTL_RESERVED_1_MAX                                     (0x0001U)

#define CSL_ADC_ADCBURSTCTL_BURSTSIZE_MASK                                     (0x0F00U)
#define CSL_ADC_ADCBURSTCTL_BURSTSIZE_SHIFT                                    (0x0008U)
#define CSL_ADC_ADCBURSTCTL_BURSTSIZE_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCBURSTCTL_BURSTSIZE_MAX                                      (0x000FU)

#define CSL_ADC_ADCBURSTCTL_RESERVED_2_MASK                                    (0x7000U)
#define CSL_ADC_ADCBURSTCTL_RESERVED_2_SHIFT                                   (0x000CU)
#define CSL_ADC_ADCBURSTCTL_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_ADC_ADCBURSTCTL_RESERVED_2_MAX                                     (0x0007U)

#define CSL_ADC_ADCBURSTCTL_BURSTEN_MASK                                       (0x8000U)
#define CSL_ADC_ADCBURSTCTL_BURSTEN_SHIFT                                      (0x000FU)
#define CSL_ADC_ADCBURSTCTL_BURSTEN_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCBURSTCTL_BURSTEN_MAX                                        (0x0001U)

#define CSL_ADC_ADCBURSTCTL_RESETVAL                                           (0x0000U)

/* ADCINTFLG */

#define CSL_ADC_ADCINTFLG_ADCINT1_MASK                                         (0x0001U)
#define CSL_ADC_ADCINTFLG_ADCINT1_SHIFT                                        (0x0000U)
#define CSL_ADC_ADCINTFLG_ADCINT1_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTFLG_ADCINT1_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTFLG_ADCINT2_MASK                                         (0x0002U)
#define CSL_ADC_ADCINTFLG_ADCINT2_SHIFT                                        (0x0001U)
#define CSL_ADC_ADCINTFLG_ADCINT2_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTFLG_ADCINT2_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTFLG_ADCINT3_MASK                                         (0x0004U)
#define CSL_ADC_ADCINTFLG_ADCINT3_SHIFT                                        (0x0002U)
#define CSL_ADC_ADCINTFLG_ADCINT3_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTFLG_ADCINT3_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTFLG_ADCINT4_MASK                                         (0x0008U)
#define CSL_ADC_ADCINTFLG_ADCINT4_SHIFT                                        (0x0003U)
#define CSL_ADC_ADCINTFLG_ADCINT4_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTFLG_ADCINT4_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTFLG_RESERVED_1_MASK                                      (0xFFF0U)
#define CSL_ADC_ADCINTFLG_RESERVED_1_SHIFT                                     (0x0004U)
#define CSL_ADC_ADCINTFLG_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTFLG_RESERVED_1_MAX                                       (0x0FFFU)

#define CSL_ADC_ADCINTFLG_RESETVAL                                             (0x0000U)

/* ADCINTFLGCLR */

#define CSL_ADC_ADCINTFLGCLR_ADCINT1_MASK                                      (0x0001U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT1_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT1_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTFLGCLR_ADCINT2_MASK                                      (0x0002U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT2_SHIFT                                     (0x0001U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT2_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT2_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTFLGCLR_ADCINT3_MASK                                      (0x0004U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT3_SHIFT                                     (0x0002U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT3_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT3_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTFLGCLR_ADCINT4_MASK                                      (0x0008U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT4_SHIFT                                     (0x0003U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT4_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_ADCINT4_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTFLGCLR_RESERVED_1_MASK                                   (0xFFF0U)
#define CSL_ADC_ADCINTFLGCLR_RESERVED_1_SHIFT                                  (0x0004U)
#define CSL_ADC_ADCINTFLGCLR_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTFLGCLR_RESERVED_1_MAX                                    (0x0FFFU)

#define CSL_ADC_ADCINTFLGCLR_RESETVAL                                          (0x0000U)

/* ADCINTOVF */

#define CSL_ADC_ADCINTOVF_ADCINT1_MASK                                         (0x0001U)
#define CSL_ADC_ADCINTOVF_ADCINT1_SHIFT                                        (0x0000U)
#define CSL_ADC_ADCINTOVF_ADCINT1_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTOVF_ADCINT1_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTOVF_ADCINT2_MASK                                         (0x0002U)
#define CSL_ADC_ADCINTOVF_ADCINT2_SHIFT                                        (0x0001U)
#define CSL_ADC_ADCINTOVF_ADCINT2_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTOVF_ADCINT2_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTOVF_ADCINT3_MASK                                         (0x0004U)
#define CSL_ADC_ADCINTOVF_ADCINT3_SHIFT                                        (0x0002U)
#define CSL_ADC_ADCINTOVF_ADCINT3_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTOVF_ADCINT3_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTOVF_ADCINT4_MASK                                         (0x0008U)
#define CSL_ADC_ADCINTOVF_ADCINT4_SHIFT                                        (0x0003U)
#define CSL_ADC_ADCINTOVF_ADCINT4_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTOVF_ADCINT4_MAX                                          (0x0001U)

#define CSL_ADC_ADCINTOVF_RESERVED_1_MASK                                      (0xFFF0U)
#define CSL_ADC_ADCINTOVF_RESERVED_1_SHIFT                                     (0x0004U)
#define CSL_ADC_ADCINTOVF_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTOVF_RESERVED_1_MAX                                       (0x0FFFU)

#define CSL_ADC_ADCINTOVF_RESETVAL                                             (0x0000U)

/* ADCINTOVFCLR */

#define CSL_ADC_ADCINTOVFCLR_ADCINT1_MASK                                      (0x0001U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT1_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT1_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTOVFCLR_ADCINT2_MASK                                      (0x0002U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT2_SHIFT                                     (0x0001U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT2_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT2_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTOVFCLR_ADCINT3_MASK                                      (0x0004U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT3_SHIFT                                     (0x0002U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT3_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT3_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTOVFCLR_ADCINT4_MASK                                      (0x0008U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT4_SHIFT                                     (0x0003U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT4_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_ADCINT4_MAX                                       (0x0001U)

#define CSL_ADC_ADCINTOVFCLR_RESERVED_1_MASK                                   (0xFFF0U)
#define CSL_ADC_ADCINTOVFCLR_RESERVED_1_SHIFT                                  (0x0004U)
#define CSL_ADC_ADCINTOVFCLR_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTOVFCLR_RESERVED_1_MAX                                    (0x0FFFU)

#define CSL_ADC_ADCINTOVFCLR_RESETVAL                                          (0x0000U)

/* ADCINTSEL1N2 */

#define CSL_ADC_ADCINTSEL1N2_INT1SEL_MASK                                      (0x000FU)
#define CSL_ADC_ADCINTSEL1N2_INT1SEL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT1SEL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT1SEL_MAX                                       (0x000FU)

#define CSL_ADC_ADCINTSEL1N2_RESERVED_1_MASK                                   (0x0010U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_1_SHIFT                                  (0x0004U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_INT1E_MASK                                        (0x0020U)
#define CSL_ADC_ADCINTSEL1N2_INT1E_SHIFT                                       (0x0005U)
#define CSL_ADC_ADCINTSEL1N2_INT1E_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT1E_MAX                                         (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_INT1CONT_MASK                                     (0x0040U)
#define CSL_ADC_ADCINTSEL1N2_INT1CONT_SHIFT                                    (0x0006U)
#define CSL_ADC_ADCINTSEL1N2_INT1CONT_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT1CONT_MAX                                      (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_RESERVED_2_MASK                                   (0x0080U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_2_SHIFT                                  (0x0007U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_2_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_INT2SEL_MASK                                      (0x0F00U)
#define CSL_ADC_ADCINTSEL1N2_INT2SEL_SHIFT                                     (0x0008U)
#define CSL_ADC_ADCINTSEL1N2_INT2SEL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT2SEL_MAX                                       (0x000FU)

#define CSL_ADC_ADCINTSEL1N2_RESERVED_3_MASK                                   (0x1000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_3_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_3_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_INT2E_MASK                                        (0x2000U)
#define CSL_ADC_ADCINTSEL1N2_INT2E_SHIFT                                       (0x000DU)
#define CSL_ADC_ADCINTSEL1N2_INT2E_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT2E_MAX                                         (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_INT2CONT_MASK                                     (0x4000U)
#define CSL_ADC_ADCINTSEL1N2_INT2CONT_SHIFT                                    (0x000EU)
#define CSL_ADC_ADCINTSEL1N2_INT2CONT_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_INT2CONT_MAX                                      (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_RESERVED_4_MASK                                   (0x8000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_4_SHIFT                                  (0x000FU)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL1N2_RESERVED_4_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL1N2_RESETVAL                                          (0x0000U)

/* ADCINTSEL3N4 */

#define CSL_ADC_ADCINTSEL3N4_INT3SEL_MASK                                      (0x000FU)
#define CSL_ADC_ADCINTSEL3N4_INT3SEL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT3SEL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT3SEL_MAX                                       (0x000FU)

#define CSL_ADC_ADCINTSEL3N4_RESERVED_1_MASK                                   (0x0010U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_1_SHIFT                                  (0x0004U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_1_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_INT3E_MASK                                        (0x0020U)
#define CSL_ADC_ADCINTSEL3N4_INT3E_SHIFT                                       (0x0005U)
#define CSL_ADC_ADCINTSEL3N4_INT3E_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT3E_MAX                                         (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_INT3CONT_MASK                                     (0x0040U)
#define CSL_ADC_ADCINTSEL3N4_INT3CONT_SHIFT                                    (0x0006U)
#define CSL_ADC_ADCINTSEL3N4_INT3CONT_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT3CONT_MAX                                      (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_RESERVED_2_MASK                                   (0x0080U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_2_SHIFT                                  (0x0007U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_2_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_INT4SEL_MASK                                      (0x0F00U)
#define CSL_ADC_ADCINTSEL3N4_INT4SEL_SHIFT                                     (0x0008U)
#define CSL_ADC_ADCINTSEL3N4_INT4SEL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT4SEL_MAX                                       (0x000FU)

#define CSL_ADC_ADCINTSEL3N4_RESERVED_3_MASK                                   (0x1000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_3_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_3_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_INT4E_MASK                                        (0x2000U)
#define CSL_ADC_ADCINTSEL3N4_INT4E_SHIFT                                       (0x000DU)
#define CSL_ADC_ADCINTSEL3N4_INT4E_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT4E_MAX                                         (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_INT4CONT_MASK                                     (0x4000U)
#define CSL_ADC_ADCINTSEL3N4_INT4CONT_SHIFT                                    (0x000EU)
#define CSL_ADC_ADCINTSEL3N4_INT4CONT_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_INT4CONT_MAX                                      (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_RESERVED_4_MASK                                   (0x8000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_4_SHIFT                                  (0x000FU)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCINTSEL3N4_RESERVED_4_MAX                                    (0x0001U)

#define CSL_ADC_ADCINTSEL3N4_RESETVAL                                          (0x0000U)

/* ADCSOCPRICTL */

#define CSL_ADC_ADCSOCPRICTL_SOCPRIORITY_MASK                                  (0x001FU)
#define CSL_ADC_ADCSOCPRICTL_SOCPRIORITY_SHIFT                                 (0x0000U)
#define CSL_ADC_ADCSOCPRICTL_SOCPRIORITY_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCSOCPRICTL_SOCPRIORITY_MAX                                   (0x001FU)

#define CSL_ADC_ADCSOCPRICTL_RRPOINTER_MASK                                    (0x03E0U)
#define CSL_ADC_ADCSOCPRICTL_RRPOINTER_SHIFT                                   (0x0005U)
#define CSL_ADC_ADCSOCPRICTL_RRPOINTER_RESETVAL                                (0x0010U)
#define CSL_ADC_ADCSOCPRICTL_RRPOINTER_MAX                                     (0x001FU)

#define CSL_ADC_ADCSOCPRICTL_RESERVED_1_MASK                                   (0xFC00U)
#define CSL_ADC_ADCSOCPRICTL_RESERVED_1_SHIFT                                  (0x000AU)
#define CSL_ADC_ADCSOCPRICTL_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCSOCPRICTL_RESERVED_1_MAX                                    (0x003FU)

#define CSL_ADC_ADCSOCPRICTL_RESETVAL                                          (0x0200U)

/* ADCINTSOCSEL1 */

#define CSL_ADC_ADCINTSOCSEL1_SOC0_MASK                                        (0x0003U)
#define CSL_ADC_ADCINTSOCSEL1_SOC0_SHIFT                                       (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC0_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC0_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC1_MASK                                        (0x000CU)
#define CSL_ADC_ADCINTSOCSEL1_SOC1_SHIFT                                       (0x0002U)
#define CSL_ADC_ADCINTSOCSEL1_SOC1_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC1_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC2_MASK                                        (0x0030U)
#define CSL_ADC_ADCINTSOCSEL1_SOC2_SHIFT                                       (0x0004U)
#define CSL_ADC_ADCINTSOCSEL1_SOC2_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC2_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC3_MASK                                        (0x00C0U)
#define CSL_ADC_ADCINTSOCSEL1_SOC3_SHIFT                                       (0x0006U)
#define CSL_ADC_ADCINTSOCSEL1_SOC3_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC3_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC4_MASK                                        (0x0300U)
#define CSL_ADC_ADCINTSOCSEL1_SOC4_SHIFT                                       (0x0008U)
#define CSL_ADC_ADCINTSOCSEL1_SOC4_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC4_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC5_MASK                                        (0x0C00U)
#define CSL_ADC_ADCINTSOCSEL1_SOC5_SHIFT                                       (0x000AU)
#define CSL_ADC_ADCINTSOCSEL1_SOC5_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC5_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC6_MASK                                        (0x3000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC6_SHIFT                                       (0x000CU)
#define CSL_ADC_ADCINTSOCSEL1_SOC6_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC6_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_SOC7_MASK                                        (0xC000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC7_SHIFT                                       (0x000EU)
#define CSL_ADC_ADCINTSOCSEL1_SOC7_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL1_SOC7_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL1_RESETVAL                                         (0x0000U)

/* ADCINTSOCSEL2 */

#define CSL_ADC_ADCINTSOCSEL2_SOC8_MASK                                        (0x0003U)
#define CSL_ADC_ADCINTSOCSEL2_SOC8_SHIFT                                       (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC8_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC8_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC9_MASK                                        (0x000CU)
#define CSL_ADC_ADCINTSOCSEL2_SOC9_SHIFT                                       (0x0002U)
#define CSL_ADC_ADCINTSOCSEL2_SOC9_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC9_MAX                                         (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC10_MASK                                       (0x0030U)
#define CSL_ADC_ADCINTSOCSEL2_SOC10_SHIFT                                      (0x0004U)
#define CSL_ADC_ADCINTSOCSEL2_SOC10_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC10_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC11_MASK                                       (0x00C0U)
#define CSL_ADC_ADCINTSOCSEL2_SOC11_SHIFT                                      (0x0006U)
#define CSL_ADC_ADCINTSOCSEL2_SOC11_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC11_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC12_MASK                                       (0x0300U)
#define CSL_ADC_ADCINTSOCSEL2_SOC12_SHIFT                                      (0x0008U)
#define CSL_ADC_ADCINTSOCSEL2_SOC12_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC12_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC13_MASK                                       (0x0C00U)
#define CSL_ADC_ADCINTSOCSEL2_SOC13_SHIFT                                      (0x000AU)
#define CSL_ADC_ADCINTSOCSEL2_SOC13_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC13_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC14_MASK                                       (0x3000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC14_SHIFT                                      (0x000CU)
#define CSL_ADC_ADCINTSOCSEL2_SOC14_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC14_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_SOC15_MASK                                       (0xC000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC15_SHIFT                                      (0x000EU)
#define CSL_ADC_ADCINTSOCSEL2_SOC15_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCINTSOCSEL2_SOC15_MAX                                        (0x0003U)

#define CSL_ADC_ADCINTSOCSEL2_RESETVAL                                         (0x0000U)

/* ADCSOCFLG1 */

#define CSL_ADC_ADCSOCFLG1_SOC0_MASK                                           (0x0001U)
#define CSL_ADC_ADCSOCFLG1_SOC0_SHIFT                                          (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC0_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC0_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC1_MASK                                           (0x0002U)
#define CSL_ADC_ADCSOCFLG1_SOC1_SHIFT                                          (0x0001U)
#define CSL_ADC_ADCSOCFLG1_SOC1_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC1_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC2_MASK                                           (0x0004U)
#define CSL_ADC_ADCSOCFLG1_SOC2_SHIFT                                          (0x0002U)
#define CSL_ADC_ADCSOCFLG1_SOC2_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC2_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC3_MASK                                           (0x0008U)
#define CSL_ADC_ADCSOCFLG1_SOC3_SHIFT                                          (0x0003U)
#define CSL_ADC_ADCSOCFLG1_SOC3_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC3_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC4_MASK                                           (0x0010U)
#define CSL_ADC_ADCSOCFLG1_SOC4_SHIFT                                          (0x0004U)
#define CSL_ADC_ADCSOCFLG1_SOC4_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC4_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC5_MASK                                           (0x0020U)
#define CSL_ADC_ADCSOCFLG1_SOC5_SHIFT                                          (0x0005U)
#define CSL_ADC_ADCSOCFLG1_SOC5_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC5_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC6_MASK                                           (0x0040U)
#define CSL_ADC_ADCSOCFLG1_SOC6_SHIFT                                          (0x0006U)
#define CSL_ADC_ADCSOCFLG1_SOC6_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC6_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC7_MASK                                           (0x0080U)
#define CSL_ADC_ADCSOCFLG1_SOC7_SHIFT                                          (0x0007U)
#define CSL_ADC_ADCSOCFLG1_SOC7_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC7_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC8_MASK                                           (0x0100U)
#define CSL_ADC_ADCSOCFLG1_SOC8_SHIFT                                          (0x0008U)
#define CSL_ADC_ADCSOCFLG1_SOC8_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC8_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC9_MASK                                           (0x0200U)
#define CSL_ADC_ADCSOCFLG1_SOC9_SHIFT                                          (0x0009U)
#define CSL_ADC_ADCSOCFLG1_SOC9_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC9_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC10_MASK                                          (0x0400U)
#define CSL_ADC_ADCSOCFLG1_SOC10_SHIFT                                         (0x000AU)
#define CSL_ADC_ADCSOCFLG1_SOC10_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC10_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC11_MASK                                          (0x0800U)
#define CSL_ADC_ADCSOCFLG1_SOC11_SHIFT                                         (0x000BU)
#define CSL_ADC_ADCSOCFLG1_SOC11_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC11_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC12_MASK                                          (0x1000U)
#define CSL_ADC_ADCSOCFLG1_SOC12_SHIFT                                         (0x000CU)
#define CSL_ADC_ADCSOCFLG1_SOC12_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC12_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC13_MASK                                          (0x2000U)
#define CSL_ADC_ADCSOCFLG1_SOC13_SHIFT                                         (0x000DU)
#define CSL_ADC_ADCSOCFLG1_SOC13_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC13_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC14_MASK                                          (0x4000U)
#define CSL_ADC_ADCSOCFLG1_SOC14_SHIFT                                         (0x000EU)
#define CSL_ADC_ADCSOCFLG1_SOC14_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC14_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_SOC15_MASK                                          (0x8000U)
#define CSL_ADC_ADCSOCFLG1_SOC15_SHIFT                                         (0x000FU)
#define CSL_ADC_ADCSOCFLG1_SOC15_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFLG1_SOC15_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFLG1_RESETVAL                                            (0x0000U)

/* ADCSOCFRC1 */

#define CSL_ADC_ADCSOCFRC1_SOC0_MASK                                           (0x0001U)
#define CSL_ADC_ADCSOCFRC1_SOC0_SHIFT                                          (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC0_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC0_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC1_MASK                                           (0x0002U)
#define CSL_ADC_ADCSOCFRC1_SOC1_SHIFT                                          (0x0001U)
#define CSL_ADC_ADCSOCFRC1_SOC1_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC1_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC2_MASK                                           (0x0004U)
#define CSL_ADC_ADCSOCFRC1_SOC2_SHIFT                                          (0x0002U)
#define CSL_ADC_ADCSOCFRC1_SOC2_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC2_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC3_MASK                                           (0x0008U)
#define CSL_ADC_ADCSOCFRC1_SOC3_SHIFT                                          (0x0003U)
#define CSL_ADC_ADCSOCFRC1_SOC3_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC3_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC4_MASK                                           (0x0010U)
#define CSL_ADC_ADCSOCFRC1_SOC4_SHIFT                                          (0x0004U)
#define CSL_ADC_ADCSOCFRC1_SOC4_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC4_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC5_MASK                                           (0x0020U)
#define CSL_ADC_ADCSOCFRC1_SOC5_SHIFT                                          (0x0005U)
#define CSL_ADC_ADCSOCFRC1_SOC5_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC5_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC6_MASK                                           (0x0040U)
#define CSL_ADC_ADCSOCFRC1_SOC6_SHIFT                                          (0x0006U)
#define CSL_ADC_ADCSOCFRC1_SOC6_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC6_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC7_MASK                                           (0x0080U)
#define CSL_ADC_ADCSOCFRC1_SOC7_SHIFT                                          (0x0007U)
#define CSL_ADC_ADCSOCFRC1_SOC7_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC7_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC8_MASK                                           (0x0100U)
#define CSL_ADC_ADCSOCFRC1_SOC8_SHIFT                                          (0x0008U)
#define CSL_ADC_ADCSOCFRC1_SOC8_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC8_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC9_MASK                                           (0x0200U)
#define CSL_ADC_ADCSOCFRC1_SOC9_SHIFT                                          (0x0009U)
#define CSL_ADC_ADCSOCFRC1_SOC9_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC9_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC10_MASK                                          (0x0400U)
#define CSL_ADC_ADCSOCFRC1_SOC10_SHIFT                                         (0x000AU)
#define CSL_ADC_ADCSOCFRC1_SOC10_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC10_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC11_MASK                                          (0x0800U)
#define CSL_ADC_ADCSOCFRC1_SOC11_SHIFT                                         (0x000BU)
#define CSL_ADC_ADCSOCFRC1_SOC11_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC11_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC12_MASK                                          (0x1000U)
#define CSL_ADC_ADCSOCFRC1_SOC12_SHIFT                                         (0x000CU)
#define CSL_ADC_ADCSOCFRC1_SOC12_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC12_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC13_MASK                                          (0x2000U)
#define CSL_ADC_ADCSOCFRC1_SOC13_SHIFT                                         (0x000DU)
#define CSL_ADC_ADCSOCFRC1_SOC13_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC13_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC14_MASK                                          (0x4000U)
#define CSL_ADC_ADCSOCFRC1_SOC14_SHIFT                                         (0x000EU)
#define CSL_ADC_ADCSOCFRC1_SOC14_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC14_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_SOC15_MASK                                          (0x8000U)
#define CSL_ADC_ADCSOCFRC1_SOC15_SHIFT                                         (0x000FU)
#define CSL_ADC_ADCSOCFRC1_SOC15_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCFRC1_SOC15_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCFRC1_RESETVAL                                            (0x0000U)

/* ADCSOCOVF1 */

#define CSL_ADC_ADCSOCOVF1_SOC0_MASK                                           (0x0001U)
#define CSL_ADC_ADCSOCOVF1_SOC0_SHIFT                                          (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC0_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC0_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC1_MASK                                           (0x0002U)
#define CSL_ADC_ADCSOCOVF1_SOC1_SHIFT                                          (0x0001U)
#define CSL_ADC_ADCSOCOVF1_SOC1_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC1_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC2_MASK                                           (0x0004U)
#define CSL_ADC_ADCSOCOVF1_SOC2_SHIFT                                          (0x0002U)
#define CSL_ADC_ADCSOCOVF1_SOC2_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC2_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC3_MASK                                           (0x0008U)
#define CSL_ADC_ADCSOCOVF1_SOC3_SHIFT                                          (0x0003U)
#define CSL_ADC_ADCSOCOVF1_SOC3_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC3_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC4_MASK                                           (0x0010U)
#define CSL_ADC_ADCSOCOVF1_SOC4_SHIFT                                          (0x0004U)
#define CSL_ADC_ADCSOCOVF1_SOC4_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC4_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC5_MASK                                           (0x0020U)
#define CSL_ADC_ADCSOCOVF1_SOC5_SHIFT                                          (0x0005U)
#define CSL_ADC_ADCSOCOVF1_SOC5_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC5_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC6_MASK                                           (0x0040U)
#define CSL_ADC_ADCSOCOVF1_SOC6_SHIFT                                          (0x0006U)
#define CSL_ADC_ADCSOCOVF1_SOC6_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC6_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC7_MASK                                           (0x0080U)
#define CSL_ADC_ADCSOCOVF1_SOC7_SHIFT                                          (0x0007U)
#define CSL_ADC_ADCSOCOVF1_SOC7_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC7_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC8_MASK                                           (0x0100U)
#define CSL_ADC_ADCSOCOVF1_SOC8_SHIFT                                          (0x0008U)
#define CSL_ADC_ADCSOCOVF1_SOC8_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC8_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC9_MASK                                           (0x0200U)
#define CSL_ADC_ADCSOCOVF1_SOC9_SHIFT                                          (0x0009U)
#define CSL_ADC_ADCSOCOVF1_SOC9_RESETVAL                                       (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC9_MAX                                            (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC10_MASK                                          (0x0400U)
#define CSL_ADC_ADCSOCOVF1_SOC10_SHIFT                                         (0x000AU)
#define CSL_ADC_ADCSOCOVF1_SOC10_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC10_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC11_MASK                                          (0x0800U)
#define CSL_ADC_ADCSOCOVF1_SOC11_SHIFT                                         (0x000BU)
#define CSL_ADC_ADCSOCOVF1_SOC11_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC11_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC12_MASK                                          (0x1000U)
#define CSL_ADC_ADCSOCOVF1_SOC12_SHIFT                                         (0x000CU)
#define CSL_ADC_ADCSOCOVF1_SOC12_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC12_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC13_MASK                                          (0x2000U)
#define CSL_ADC_ADCSOCOVF1_SOC13_SHIFT                                         (0x000DU)
#define CSL_ADC_ADCSOCOVF1_SOC13_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC13_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC14_MASK                                          (0x4000U)
#define CSL_ADC_ADCSOCOVF1_SOC14_SHIFT                                         (0x000EU)
#define CSL_ADC_ADCSOCOVF1_SOC14_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC14_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_SOC15_MASK                                          (0x8000U)
#define CSL_ADC_ADCSOCOVF1_SOC15_SHIFT                                         (0x000FU)
#define CSL_ADC_ADCSOCOVF1_SOC15_RESETVAL                                      (0x0000U)
#define CSL_ADC_ADCSOCOVF1_SOC15_MAX                                           (0x0001U)

#define CSL_ADC_ADCSOCOVF1_RESETVAL                                            (0x0000U)

/* ADCSOCOVFCLR1 */

#define CSL_ADC_ADCSOCOVFCLR1_SOC0_MASK                                        (0x0001U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC0_SHIFT                                       (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC0_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC0_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC1_MASK                                        (0x0002U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC1_SHIFT                                       (0x0001U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC1_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC1_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC2_MASK                                        (0x0004U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC2_SHIFT                                       (0x0002U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC2_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC2_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC3_MASK                                        (0x0008U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC3_SHIFT                                       (0x0003U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC3_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC3_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC4_MASK                                        (0x0010U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC4_SHIFT                                       (0x0004U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC4_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC4_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC5_MASK                                        (0x0020U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC5_SHIFT                                       (0x0005U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC5_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC5_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC6_MASK                                        (0x0040U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC6_SHIFT                                       (0x0006U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC6_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC6_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC7_MASK                                        (0x0080U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC7_SHIFT                                       (0x0007U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC7_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC7_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC8_MASK                                        (0x0100U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC8_SHIFT                                       (0x0008U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC8_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC8_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC9_MASK                                        (0x0200U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC9_SHIFT                                       (0x0009U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC9_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC9_MAX                                         (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC10_MASK                                       (0x0400U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC10_SHIFT                                      (0x000AU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC10_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC10_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC11_MASK                                       (0x0800U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC11_SHIFT                                      (0x000BU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC11_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC11_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC12_MASK                                       (0x1000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC12_SHIFT                                      (0x000CU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC12_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC12_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC13_MASK                                       (0x2000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC13_SHIFT                                      (0x000DU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC13_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC13_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC14_MASK                                       (0x4000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC14_SHIFT                                      (0x000EU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC14_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC14_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_SOC15_MASK                                       (0x8000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC15_SHIFT                                      (0x000FU)
#define CSL_ADC_ADCSOCOVFCLR1_SOC15_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCSOCOVFCLR1_SOC15_MAX                                        (0x0001U)

#define CSL_ADC_ADCSOCOVFCLR1_RESETVAL                                         (0x0000U)

/* ADCSOC0CTL */

#define CSL_ADC_ADCSOC0CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC0CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC0CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC0CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC0CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC0CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC0CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC0CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC0CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC0CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC0CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC0CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC0CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC0CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC0CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC0CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC1CTL */

#define CSL_ADC_ADCSOC1CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC1CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC1CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC1CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC1CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC1CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC1CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC1CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC1CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC1CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC1CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC1CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC1CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC1CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC1CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC1CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC2CTL */

#define CSL_ADC_ADCSOC2CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC2CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC2CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC2CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC2CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC2CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC2CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC2CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC2CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC2CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC2CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC2CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC2CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC2CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC2CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC2CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC3CTL */

#define CSL_ADC_ADCSOC3CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC3CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC3CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC3CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC3CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC3CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC3CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC3CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC3CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC3CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC3CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC3CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC3CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC3CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC3CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC3CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC4CTL */

#define CSL_ADC_ADCSOC4CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC4CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC4CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC4CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC4CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC4CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC4CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC4CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC4CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC4CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC4CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC4CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC4CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC4CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC4CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC4CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC5CTL */

#define CSL_ADC_ADCSOC5CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC5CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC5CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC5CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC5CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC5CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC5CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC5CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC5CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC5CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC5CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC5CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC5CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC5CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC5CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC5CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC6CTL */

#define CSL_ADC_ADCSOC6CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC6CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC6CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC6CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC6CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC6CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC6CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC6CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC6CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC6CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC6CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC6CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC6CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC6CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC6CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC6CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC7CTL */

#define CSL_ADC_ADCSOC7CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC7CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC7CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC7CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC7CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC7CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC7CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC7CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC7CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC7CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC7CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC7CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC7CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC7CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC7CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC7CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC8CTL */

#define CSL_ADC_ADCSOC8CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC8CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC8CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC8CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC8CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC8CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC8CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC8CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC8CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC8CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC8CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC8CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC8CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC8CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC8CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC8CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC9CTL */

#define CSL_ADC_ADCSOC9CTL_ACQPS_MASK                                          (0x000001FFU)
#define CSL_ADC_ADCSOC9CTL_ACQPS_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_ACQPS_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_ACQPS_MAX                                           (0x000001FFU)

#define CSL_ADC_ADCSOC9CTL_RESERVED_1_MASK                                     (0x00007E00U)
#define CSL_ADC_ADCSOC9CTL_RESERVED_1_SHIFT                                    (0x00000009U)
#define CSL_ADC_ADCSOC9CTL_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_RESERVED_1_MAX                                      (0x0000003FU)

#define CSL_ADC_ADCSOC9CTL_CHSEL_MASK                                          (0x000F8000U)
#define CSL_ADC_ADCSOC9CTL_CHSEL_SHIFT                                         (0x0000000FU)
#define CSL_ADC_ADCSOC9CTL_CHSEL_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_CHSEL_MAX                                           (0x0000001FU)

#define CSL_ADC_ADCSOC9CTL_TRIGSEL_MASK                                        (0x07F00000U)
#define CSL_ADC_ADCSOC9CTL_TRIGSEL_SHIFT                                       (0x00000014U)
#define CSL_ADC_ADCSOC9CTL_TRIGSEL_RESETVAL                                    (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_TRIGSEL_MAX                                         (0x0000007FU)

#define CSL_ADC_ADCSOC9CTL_RESERVED_2_MASK                                     (0xF8000000U)
#define CSL_ADC_ADCSOC9CTL_RESERVED_2_SHIFT                                    (0x0000001BU)
#define CSL_ADC_ADCSOC9CTL_RESERVED_2_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCSOC9CTL_RESERVED_2_MAX                                      (0x0000001FU)

#define CSL_ADC_ADCSOC9CTL_RESETVAL                                            (0x00000000U)

/* ADCSOC10CTL */

#define CSL_ADC_ADCSOC10CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC10CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC10CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC10CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC10CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC10CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC10CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC10CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC10CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC10CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC10CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC10CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC10CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC10CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC10CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC10CTL_RESETVAL                                           (0x00000000U)

/* ADCSOC11CTL */

#define CSL_ADC_ADCSOC11CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC11CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC11CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC11CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC11CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC11CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC11CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC11CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC11CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC11CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC11CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC11CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC11CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC11CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC11CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC11CTL_RESETVAL                                           (0x00000000U)

/* ADCSOC12CTL */

#define CSL_ADC_ADCSOC12CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC12CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC12CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC12CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC12CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC12CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC12CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC12CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC12CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC12CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC12CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC12CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC12CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC12CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC12CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC12CTL_RESETVAL                                           (0x00000000U)

/* ADCSOC13CTL */

#define CSL_ADC_ADCSOC13CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC13CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC13CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC13CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC13CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC13CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC13CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC13CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC13CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC13CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC13CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC13CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC13CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC13CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC13CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC13CTL_RESETVAL                                           (0x00000000U)

/* ADCSOC14CTL */

#define CSL_ADC_ADCSOC14CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC14CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC14CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC14CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC14CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC14CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC14CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC14CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC14CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC14CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC14CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC14CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC14CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC14CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC14CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC14CTL_RESETVAL                                           (0x00000000U)

/* ADCSOC15CTL */

#define CSL_ADC_ADCSOC15CTL_ACQPS_MASK                                         (0x000001FFU)
#define CSL_ADC_ADCSOC15CTL_ACQPS_SHIFT                                        (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_ACQPS_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_ACQPS_MAX                                          (0x000001FFU)

#define CSL_ADC_ADCSOC15CTL_RESERVED_1_MASK                                    (0x00007E00U)
#define CSL_ADC_ADCSOC15CTL_RESERVED_1_SHIFT                                   (0x00000009U)
#define CSL_ADC_ADCSOC15CTL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_RESERVED_1_MAX                                     (0x0000003FU)

#define CSL_ADC_ADCSOC15CTL_CHSEL_MASK                                         (0x000F8000U)
#define CSL_ADC_ADCSOC15CTL_CHSEL_SHIFT                                        (0x0000000FU)
#define CSL_ADC_ADCSOC15CTL_CHSEL_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_CHSEL_MAX                                          (0x0000001FU)

#define CSL_ADC_ADCSOC15CTL_TRIGSEL_MASK                                       (0x07F00000U)
#define CSL_ADC_ADCSOC15CTL_TRIGSEL_SHIFT                                      (0x00000014U)
#define CSL_ADC_ADCSOC15CTL_TRIGSEL_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_TRIGSEL_MAX                                        (0x0000007FU)

#define CSL_ADC_ADCSOC15CTL_RESERVED_2_MASK                                    (0xF8000000U)
#define CSL_ADC_ADCSOC15CTL_RESERVED_2_SHIFT                                   (0x0000001BU)
#define CSL_ADC_ADCSOC15CTL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCSOC15CTL_RESERVED_2_MAX                                     (0x0000001FU)

#define CSL_ADC_ADCSOC15CTL_RESETVAL                                           (0x00000000U)

/* ADCEVTSTAT */

#define CSL_ADC_ADCEVTSTAT_PPB1TRIPHI_MASK                                     (0x0001U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPHI_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPHI_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPHI_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB1TRIPLO_MASK                                     (0x0002U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPLO_SHIFT                                    (0x0001U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPLO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB1TRIPLO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB1ZERO_MASK                                       (0x0004U)
#define CSL_ADC_ADCEVTSTAT_PPB1ZERO_SHIFT                                      (0x0002U)
#define CSL_ADC_ADCEVTSTAT_PPB1ZERO_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB1ZERO_MAX                                        (0x0001U)

#define CSL_ADC_ADCEVTSTAT_RESERVED_1_MASK                                     (0x0008U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_1_SHIFT                                    (0x0003U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_1_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB2TRIPHI_MASK                                     (0x0010U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPHI_SHIFT                                    (0x0004U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPHI_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPHI_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB2TRIPLO_MASK                                     (0x0020U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPLO_SHIFT                                    (0x0005U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPLO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB2TRIPLO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB2ZERO_MASK                                       (0x0040U)
#define CSL_ADC_ADCEVTSTAT_PPB2ZERO_SHIFT                                      (0x0006U)
#define CSL_ADC_ADCEVTSTAT_PPB2ZERO_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB2ZERO_MAX                                        (0x0001U)

#define CSL_ADC_ADCEVTSTAT_RESERVED_2_MASK                                     (0x0080U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_2_SHIFT                                    (0x0007U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_2_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_2_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB3TRIPHI_MASK                                     (0x0100U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPHI_SHIFT                                    (0x0008U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPHI_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPHI_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB3TRIPLO_MASK                                     (0x0200U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPLO_SHIFT                                    (0x0009U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPLO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB3TRIPLO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB3ZERO_MASK                                       (0x0400U)
#define CSL_ADC_ADCEVTSTAT_PPB3ZERO_SHIFT                                      (0x000AU)
#define CSL_ADC_ADCEVTSTAT_PPB3ZERO_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB3ZERO_MAX                                        (0x0001U)

#define CSL_ADC_ADCEVTSTAT_RESERVED_3_MASK                                     (0x0800U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_3_SHIFT                                    (0x000BU)
#define CSL_ADC_ADCEVTSTAT_RESERVED_3_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_3_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB4TRIPHI_MASK                                     (0x1000U)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPHI_SHIFT                                    (0x000CU)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPHI_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPHI_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB4TRIPLO_MASK                                     (0x2000U)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPLO_SHIFT                                    (0x000DU)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPLO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB4TRIPLO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_PPB4ZERO_MASK                                       (0x4000U)
#define CSL_ADC_ADCEVTSTAT_PPB4ZERO_SHIFT                                      (0x000EU)
#define CSL_ADC_ADCEVTSTAT_PPB4ZERO_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCEVTSTAT_PPB4ZERO_MAX                                        (0x0001U)

#define CSL_ADC_ADCEVTSTAT_RESERVED_4_MASK                                     (0x8000U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_4_SHIFT                                    (0x000FU)
#define CSL_ADC_ADCEVTSTAT_RESERVED_4_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTSTAT_RESERVED_4_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTSTAT_RESETVAL                                            (0x0000U)

/* ADCEVTCLR */

#define CSL_ADC_ADCEVTCLR_PPB1TRIPHI_MASK                                      (0x0001U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPHI_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB1TRIPLO_MASK                                      (0x0002U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPLO_SHIFT                                     (0x0001U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB1TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB1ZERO_MASK                                        (0x0004U)
#define CSL_ADC_ADCEVTCLR_PPB1ZERO_SHIFT                                       (0x0002U)
#define CSL_ADC_ADCEVTCLR_PPB1ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB1ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTCLR_RESERVED_1_MASK                                      (0x0008U)
#define CSL_ADC_ADCEVTCLR_RESERVED_1_SHIFT                                     (0x0003U)
#define CSL_ADC_ADCEVTCLR_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_RESERVED_1_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB2TRIPHI_MASK                                      (0x0010U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPHI_SHIFT                                     (0x0004U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB2TRIPLO_MASK                                      (0x0020U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPLO_SHIFT                                     (0x0005U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB2TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB2ZERO_MASK                                        (0x0040U)
#define CSL_ADC_ADCEVTCLR_PPB2ZERO_SHIFT                                       (0x0006U)
#define CSL_ADC_ADCEVTCLR_PPB2ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB2ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTCLR_RESERVED_2_MASK                                      (0x0080U)
#define CSL_ADC_ADCEVTCLR_RESERVED_2_SHIFT                                     (0x0007U)
#define CSL_ADC_ADCEVTCLR_RESERVED_2_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_RESERVED_2_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB3TRIPHI_MASK                                      (0x0100U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPHI_SHIFT                                     (0x0008U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB3TRIPLO_MASK                                      (0x0200U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPLO_SHIFT                                     (0x0009U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB3TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB3ZERO_MASK                                        (0x0400U)
#define CSL_ADC_ADCEVTCLR_PPB3ZERO_SHIFT                                       (0x000AU)
#define CSL_ADC_ADCEVTCLR_PPB3ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB3ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTCLR_RESERVED_3_MASK                                      (0x0800U)
#define CSL_ADC_ADCEVTCLR_RESERVED_3_SHIFT                                     (0x000BU)
#define CSL_ADC_ADCEVTCLR_RESERVED_3_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_RESERVED_3_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB4TRIPHI_MASK                                      (0x1000U)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPHI_SHIFT                                     (0x000CU)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB4TRIPLO_MASK                                      (0x2000U)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPLO_SHIFT                                     (0x000DU)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB4TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_PPB4ZERO_MASK                                        (0x4000U)
#define CSL_ADC_ADCEVTCLR_PPB4ZERO_SHIFT                                       (0x000EU)
#define CSL_ADC_ADCEVTCLR_PPB4ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTCLR_PPB4ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTCLR_RESERVED_4_MASK                                      (0x8000U)
#define CSL_ADC_ADCEVTCLR_RESERVED_4_SHIFT                                     (0x000FU)
#define CSL_ADC_ADCEVTCLR_RESERVED_4_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTCLR_RESERVED_4_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTCLR_RESETVAL                                             (0x0000U)

/* ADCEVTSEL */

#define CSL_ADC_ADCEVTSEL_PPB1TRIPHI_MASK                                      (0x0001U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPHI_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB1TRIPLO_MASK                                      (0x0002U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPLO_SHIFT                                     (0x0001U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB1TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB1ZERO_MASK                                        (0x0004U)
#define CSL_ADC_ADCEVTSEL_PPB1ZERO_SHIFT                                       (0x0002U)
#define CSL_ADC_ADCEVTSEL_PPB1ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB1ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTSEL_RESERVED_1_MASK                                      (0x0008U)
#define CSL_ADC_ADCEVTSEL_RESERVED_1_SHIFT                                     (0x0003U)
#define CSL_ADC_ADCEVTSEL_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_RESERVED_1_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB2TRIPHI_MASK                                      (0x0010U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPHI_SHIFT                                     (0x0004U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB2TRIPLO_MASK                                      (0x0020U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPLO_SHIFT                                     (0x0005U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB2TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB2ZERO_MASK                                        (0x0040U)
#define CSL_ADC_ADCEVTSEL_PPB2ZERO_SHIFT                                       (0x0006U)
#define CSL_ADC_ADCEVTSEL_PPB2ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB2ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTSEL_RESERVED_2_MASK                                      (0x0080U)
#define CSL_ADC_ADCEVTSEL_RESERVED_2_SHIFT                                     (0x0007U)
#define CSL_ADC_ADCEVTSEL_RESERVED_2_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_RESERVED_2_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB3TRIPHI_MASK                                      (0x0100U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPHI_SHIFT                                     (0x0008U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB3TRIPLO_MASK                                      (0x0200U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPLO_SHIFT                                     (0x0009U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB3TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB3ZERO_MASK                                        (0x0400U)
#define CSL_ADC_ADCEVTSEL_PPB3ZERO_SHIFT                                       (0x000AU)
#define CSL_ADC_ADCEVTSEL_PPB3ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB3ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTSEL_RESERVED_3_MASK                                      (0x0800U)
#define CSL_ADC_ADCEVTSEL_RESERVED_3_SHIFT                                     (0x000BU)
#define CSL_ADC_ADCEVTSEL_RESERVED_3_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_RESERVED_3_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB4TRIPHI_MASK                                      (0x1000U)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPHI_SHIFT                                     (0x000CU)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPHI_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPHI_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB4TRIPLO_MASK                                      (0x2000U)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPLO_SHIFT                                     (0x000DU)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPLO_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB4TRIPLO_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_PPB4ZERO_MASK                                        (0x4000U)
#define CSL_ADC_ADCEVTSEL_PPB4ZERO_SHIFT                                       (0x000EU)
#define CSL_ADC_ADCEVTSEL_PPB4ZERO_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCEVTSEL_PPB4ZERO_MAX                                         (0x0001U)

#define CSL_ADC_ADCEVTSEL_RESERVED_4_MASK                                      (0x8000U)
#define CSL_ADC_ADCEVTSEL_RESERVED_4_SHIFT                                     (0x000FU)
#define CSL_ADC_ADCEVTSEL_RESERVED_4_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCEVTSEL_RESERVED_4_MAX                                       (0x0001U)

#define CSL_ADC_ADCEVTSEL_RESETVAL                                             (0x0000U)

/* ADCEVTINTSEL */

#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPHI_MASK                                   (0x0001U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPHI_SHIFT                                  (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPHI_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPHI_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPLO_MASK                                   (0x0002U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPLO_SHIFT                                  (0x0001U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPLO_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB1TRIPLO_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB1ZERO_MASK                                     (0x0004U)
#define CSL_ADC_ADCEVTINTSEL_PPB1ZERO_SHIFT                                    (0x0002U)
#define CSL_ADC_ADCEVTINTSEL_PPB1ZERO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB1ZERO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_RESERVED_1_MASK                                   (0x0008U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_1_SHIFT                                  (0x0003U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_1_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPHI_MASK                                   (0x0010U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPHI_SHIFT                                  (0x0004U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPHI_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPHI_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPLO_MASK                                   (0x0020U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPLO_SHIFT                                  (0x0005U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPLO_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB2TRIPLO_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB2ZERO_MASK                                     (0x0040U)
#define CSL_ADC_ADCEVTINTSEL_PPB2ZERO_SHIFT                                    (0x0006U)
#define CSL_ADC_ADCEVTINTSEL_PPB2ZERO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB2ZERO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_RESERVED_2_MASK                                   (0x0080U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_2_SHIFT                                  (0x0007U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_2_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPHI_MASK                                   (0x0100U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPHI_SHIFT                                  (0x0008U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPHI_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPHI_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPLO_MASK                                   (0x0200U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPLO_SHIFT                                  (0x0009U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPLO_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB3TRIPLO_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB3ZERO_MASK                                     (0x0400U)
#define CSL_ADC_ADCEVTINTSEL_PPB3ZERO_SHIFT                                    (0x000AU)
#define CSL_ADC_ADCEVTINTSEL_PPB3ZERO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB3ZERO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_RESERVED_3_MASK                                   (0x0800U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_3_SHIFT                                  (0x000BU)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_3_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPHI_MASK                                   (0x1000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPHI_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPHI_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPHI_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPLO_MASK                                   (0x2000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPLO_SHIFT                                  (0x000DU)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPLO_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4TRIPLO_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_PPB4ZERO_MASK                                     (0x4000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4ZERO_SHIFT                                    (0x000EU)
#define CSL_ADC_ADCEVTINTSEL_PPB4ZERO_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_PPB4ZERO_MAX                                      (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_RESERVED_4_MASK                                   (0x8000U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_4_SHIFT                                  (0x000FU)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCEVTINTSEL_RESERVED_4_MAX                                    (0x0001U)

#define CSL_ADC_ADCEVTINTSEL_RESETVAL                                          (0x0000U)

/* ADCOSDETECT */

#define CSL_ADC_ADCOSDETECT_DETECTCFG_MASK                                     (0x0007U)
#define CSL_ADC_ADCOSDETECT_DETECTCFG_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCOSDETECT_DETECTCFG_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCOSDETECT_DETECTCFG_MAX                                      (0x0007U)

#define CSL_ADC_ADCOSDETECT_RESERVED_1_MASK                                    (0xFFF8U)
#define CSL_ADC_ADCOSDETECT_RESERVED_1_SHIFT                                   (0x0003U)
#define CSL_ADC_ADCOSDETECT_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_ADC_ADCOSDETECT_RESERVED_1_MAX                                     (0x1FFFU)

#define CSL_ADC_ADCOSDETECT_RESETVAL                                           (0x0000U)

/* ADCCOUNTER */

#define CSL_ADC_ADCCOUNTER_FREECOUNT_MASK                                      (0x0FFFU)
#define CSL_ADC_ADCCOUNTER_FREECOUNT_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCCOUNTER_FREECOUNT_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCCOUNTER_FREECOUNT_MAX                                       (0x0FFFU)

#define CSL_ADC_ADCCOUNTER_RESERVED_1_MASK                                     (0xF000U)
#define CSL_ADC_ADCCOUNTER_RESERVED_1_SHIFT                                    (0x000CU)
#define CSL_ADC_ADCCOUNTER_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCCOUNTER_RESERVED_1_MAX                                      (0x000FU)

#define CSL_ADC_ADCCOUNTER_RESETVAL                                            (0x0000U)

/* ADCREV */

#define CSL_ADC_ADCREV_TYPE_MASK                                               (0x00FFU)
#define CSL_ADC_ADCREV_TYPE_SHIFT                                              (0x0000U)
#define CSL_ADC_ADCREV_TYPE_RESETVAL                                           (0x0005U)
#define CSL_ADC_ADCREV_TYPE_MAX                                                (0x00FFU)

#define CSL_ADC_ADCREV_REV_MASK                                                (0xFF00U)
#define CSL_ADC_ADCREV_REV_SHIFT                                               (0x0008U)
#define CSL_ADC_ADCREV_REV_RESETVAL                                            (0x0001U)
#define CSL_ADC_ADCREV_REV_MAX                                                 (0x00FFU)

#define CSL_ADC_ADCREV_RESETVAL                                                (0x0105U)

/* ADCOFFTRIM */

#define CSL_ADC_ADCOFFTRIM_OFFTRIM_MASK                                        (0x00FFU)
#define CSL_ADC_ADCOFFTRIM_OFFTRIM_SHIFT                                       (0x0000U)
#define CSL_ADC_ADCOFFTRIM_OFFTRIM_RESETVAL                                    (0x0000U)
#define CSL_ADC_ADCOFFTRIM_OFFTRIM_MAX                                         (0x00FFU)

#define CSL_ADC_ADCOFFTRIM_RESERVED_1_MASK                                     (0xFF00U)
#define CSL_ADC_ADCOFFTRIM_RESERVED_1_SHIFT                                    (0x0008U)
#define CSL_ADC_ADCOFFTRIM_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCOFFTRIM_RESERVED_1_MAX                                      (0x00FFU)

#define CSL_ADC_ADCOFFTRIM_RESETVAL                                            (0x0000U)

/* ADCCONFIG */

#define CSL_ADC_ADCCONFIG_CONFIG_MASK                                          (0xFFFFFFFFU)
#define CSL_ADC_ADCCONFIG_CONFIG_SHIFT                                         (0x00000000U)
#define CSL_ADC_ADCCONFIG_CONFIG_RESETVAL                                      (0x00000000U)
#define CSL_ADC_ADCCONFIG_CONFIG_MAX                                           (0xFFFFFFFFU)

#define CSL_ADC_ADCCONFIG_RESETVAL                                             (0x00000000U)

/* ADCPPB1CONFIG */

#define CSL_ADC_ADCPPB1CONFIG_CONFIG_MASK                                      (0x000FU)
#define CSL_ADC_ADCPPB1CONFIG_CONFIG_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB1CONFIG_CONFIG_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB1CONFIG_CONFIG_MAX                                       (0x000FU)

#define CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_MASK                                  (0x0010U)
#define CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_SHIFT                                 (0x0004U)
#define CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_MAX                                   (0x0001U)

#define CSL_ADC_ADCPPB1CONFIG_CBCEN_MASK                                       (0x0020U)
#define CSL_ADC_ADCPPB1CONFIG_CBCEN_SHIFT                                      (0x0005U)
#define CSL_ADC_ADCPPB1CONFIG_CBCEN_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCPPB1CONFIG_CBCEN_MAX                                        (0x0001U)

#define CSL_ADC_ADCPPB1CONFIG_RESERVED_1_MASK                                  (0xFFC0U)
#define CSL_ADC_ADCPPB1CONFIG_RESERVED_1_SHIFT                                 (0x0006U)
#define CSL_ADC_ADCPPB1CONFIG_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB1CONFIG_RESERVED_1_MAX                                   (0x03FFU)

#define CSL_ADC_ADCPPB1CONFIG_RESETVAL                                         (0x0000U)

/* ADCPPB1STAMP */

#define CSL_ADC_ADCPPB1STAMP_DLYSTAMP_MASK                                     (0x0FFFU)
#define CSL_ADC_ADCPPB1STAMP_DLYSTAMP_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCPPB1STAMP_DLYSTAMP_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCPPB1STAMP_DLYSTAMP_MAX                                      (0x0FFFU)

#define CSL_ADC_ADCPPB1STAMP_RESERVED_1_MASK                                   (0xF000U)
#define CSL_ADC_ADCPPB1STAMP_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCPPB1STAMP_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCPPB1STAMP_RESERVED_1_MAX                                    (0x000FU)

#define CSL_ADC_ADCPPB1STAMP_RESETVAL                                          (0x0000U)

/* ADCPPB1OFFCAL */

#define CSL_ADC_ADCPPB1OFFCAL_OFFCAL_MASK                                      (0x03FFU)
#define CSL_ADC_ADCPPB1OFFCAL_OFFCAL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB1OFFCAL_OFFCAL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB1OFFCAL_OFFCAL_MAX                                       (0x03FFU)

#define CSL_ADC_ADCPPB1OFFCAL_RESERVED_1_MASK                                  (0xFC00U)
#define CSL_ADC_ADCPPB1OFFCAL_RESERVED_1_SHIFT                                 (0x000AU)
#define CSL_ADC_ADCPPB1OFFCAL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB1OFFCAL_RESERVED_1_MAX                                   (0x003FU)

#define CSL_ADC_ADCPPB1OFFCAL_RESETVAL                                         (0x0000U)

/* ADCPPB1OFFREF */

#define CSL_ADC_ADCPPB1OFFREF_OFFREF_MASK                                      (0xFFFFU)
#define CSL_ADC_ADCPPB1OFFREF_OFFREF_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB1OFFREF_OFFREF_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB1OFFREF_OFFREF_MAX                                       (0xFFFFU)

#define CSL_ADC_ADCPPB1OFFREF_RESETVAL                                         (0x0000U)

/* ADCPPB1TRIPHI */

#define CSL_ADC_ADCPPB1TRIPHI_LIMITHI_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB1TRIPHI_LIMITHI_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPHI_LIMITHI_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPHI_LIMITHI_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB1TRIPHI_HSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB1TRIPHI_HSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB1TRIPHI_HSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPHI_HSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB1TRIPHI_RESERVED_1_MASK                                  (0xFFFE0000U)
#define CSL_ADC_ADCPPB1TRIPHI_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB1TRIPHI_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPHI_RESERVED_1_MAX                                   (0x00007FFFU)

#define CSL_ADC_ADCPPB1TRIPHI_RESETVAL                                         (0x00000000U)

/* ADCPPB1TRIPLO */

#define CSL_ADC_ADCPPB1TRIPLO_LIMITLO_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB1TRIPLO_LIMITLO_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPLO_LIMITLO_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPLO_LIMITLO_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB1TRIPLO_LSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB1TRIPLO_LSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB1TRIPLO_LSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPLO_LSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB1TRIPLO_RESERVED_1_MASK                                  (0x000E0000U)
#define CSL_ADC_ADCPPB1TRIPLO_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB1TRIPLO_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPLO_RESERVED_1_MAX                                   (0x00000007U)

#define CSL_ADC_ADCPPB1TRIPLO_REQSTAMP_MASK                                    (0xFFF00000U)
#define CSL_ADC_ADCPPB1TRIPLO_REQSTAMP_SHIFT                                   (0x00000014U)
#define CSL_ADC_ADCPPB1TRIPLO_REQSTAMP_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCPPB1TRIPLO_REQSTAMP_MAX                                     (0x00000FFFU)

#define CSL_ADC_ADCPPB1TRIPLO_RESETVAL                                         (0x00000000U)

/* ADCPPB2CONFIG */

#define CSL_ADC_ADCPPB2CONFIG_CONFIG_MASK                                      (0x000FU)
#define CSL_ADC_ADCPPB2CONFIG_CONFIG_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB2CONFIG_CONFIG_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB2CONFIG_CONFIG_MAX                                       (0x000FU)

#define CSL_ADC_ADCPPB2CONFIG_TWOSCOMPEN_MASK                                  (0x0010U)
#define CSL_ADC_ADCPPB2CONFIG_TWOSCOMPEN_SHIFT                                 (0x0004U)
#define CSL_ADC_ADCPPB2CONFIG_TWOSCOMPEN_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB2CONFIG_TWOSCOMPEN_MAX                                   (0x0001U)

#define CSL_ADC_ADCPPB2CONFIG_CBCEN_MASK                                       (0x0020U)
#define CSL_ADC_ADCPPB2CONFIG_CBCEN_SHIFT                                      (0x0005U)
#define CSL_ADC_ADCPPB2CONFIG_CBCEN_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCPPB2CONFIG_CBCEN_MAX                                        (0x0001U)

#define CSL_ADC_ADCPPB2CONFIG_RESERVED_1_MASK                                  (0xFFC0U)
#define CSL_ADC_ADCPPB2CONFIG_RESERVED_1_SHIFT                                 (0x0006U)
#define CSL_ADC_ADCPPB2CONFIG_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB2CONFIG_RESERVED_1_MAX                                   (0x03FFU)

#define CSL_ADC_ADCPPB2CONFIG_RESETVAL                                         (0x0000U)

/* ADCPPB2STAMP */

#define CSL_ADC_ADCPPB2STAMP_DLYSTAMP_MASK                                     (0x0FFFU)
#define CSL_ADC_ADCPPB2STAMP_DLYSTAMP_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCPPB2STAMP_DLYSTAMP_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCPPB2STAMP_DLYSTAMP_MAX                                      (0x0FFFU)

#define CSL_ADC_ADCPPB2STAMP_RESERVED_1_MASK                                   (0xF000U)
#define CSL_ADC_ADCPPB2STAMP_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCPPB2STAMP_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCPPB2STAMP_RESERVED_1_MAX                                    (0x000FU)

#define CSL_ADC_ADCPPB2STAMP_RESETVAL                                          (0x0000U)

/* ADCPPB2OFFCAL */

#define CSL_ADC_ADCPPB2OFFCAL_OFFCAL_MASK                                      (0x03FFU)
#define CSL_ADC_ADCPPB2OFFCAL_OFFCAL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB2OFFCAL_OFFCAL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB2OFFCAL_OFFCAL_MAX                                       (0x03FFU)

#define CSL_ADC_ADCPPB2OFFCAL_RESERVED_1_MASK                                  (0xFC00U)
#define CSL_ADC_ADCPPB2OFFCAL_RESERVED_1_SHIFT                                 (0x000AU)
#define CSL_ADC_ADCPPB2OFFCAL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB2OFFCAL_RESERVED_1_MAX                                   (0x003FU)

#define CSL_ADC_ADCPPB2OFFCAL_RESETVAL                                         (0x0000U)

/* ADCPPB2OFFREF */

#define CSL_ADC_ADCPPB2OFFREF_OFFREF_MASK                                      (0xFFFFU)
#define CSL_ADC_ADCPPB2OFFREF_OFFREF_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB2OFFREF_OFFREF_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB2OFFREF_OFFREF_MAX                                       (0xFFFFU)

#define CSL_ADC_ADCPPB2OFFREF_RESETVAL                                         (0x0000U)

/* ADCPPB2TRIPHI */

#define CSL_ADC_ADCPPB2TRIPHI_LIMITHI_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB2TRIPHI_LIMITHI_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPHI_LIMITHI_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPHI_LIMITHI_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB2TRIPHI_HSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB2TRIPHI_HSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB2TRIPHI_HSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPHI_HSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB2TRIPHI_RESERVED_1_MASK                                  (0xFFFE0000U)
#define CSL_ADC_ADCPPB2TRIPHI_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB2TRIPHI_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPHI_RESERVED_1_MAX                                   (0x00007FFFU)

#define CSL_ADC_ADCPPB2TRIPHI_RESETVAL                                         (0x00000000U)

/* ADCPPB2TRIPLO */

#define CSL_ADC_ADCPPB2TRIPLO_LIMITLO_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB2TRIPLO_LIMITLO_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPLO_LIMITLO_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPLO_LIMITLO_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB2TRIPLO_LSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB2TRIPLO_LSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB2TRIPLO_LSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPLO_LSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB2TRIPLO_RESERVED_1_MASK                                  (0x000E0000U)
#define CSL_ADC_ADCPPB2TRIPLO_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB2TRIPLO_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPLO_RESERVED_1_MAX                                   (0x00000007U)

#define CSL_ADC_ADCPPB2TRIPLO_REQSTAMP_MASK                                    (0xFFF00000U)
#define CSL_ADC_ADCPPB2TRIPLO_REQSTAMP_SHIFT                                   (0x00000014U)
#define CSL_ADC_ADCPPB2TRIPLO_REQSTAMP_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCPPB2TRIPLO_REQSTAMP_MAX                                     (0x00000FFFU)

#define CSL_ADC_ADCPPB2TRIPLO_RESETVAL                                         (0x00000000U)

/* ADCPPB3CONFIG */

#define CSL_ADC_ADCPPB3CONFIG_CONFIG_MASK                                      (0x000FU)
#define CSL_ADC_ADCPPB3CONFIG_CONFIG_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB3CONFIG_CONFIG_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB3CONFIG_CONFIG_MAX                                       (0x000FU)

#define CSL_ADC_ADCPPB3CONFIG_TWOSCOMPEN_MASK                                  (0x0010U)
#define CSL_ADC_ADCPPB3CONFIG_TWOSCOMPEN_SHIFT                                 (0x0004U)
#define CSL_ADC_ADCPPB3CONFIG_TWOSCOMPEN_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB3CONFIG_TWOSCOMPEN_MAX                                   (0x0001U)

#define CSL_ADC_ADCPPB3CONFIG_CBCEN_MASK                                       (0x0020U)
#define CSL_ADC_ADCPPB3CONFIG_CBCEN_SHIFT                                      (0x0005U)
#define CSL_ADC_ADCPPB3CONFIG_CBCEN_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCPPB3CONFIG_CBCEN_MAX                                        (0x0001U)

#define CSL_ADC_ADCPPB3CONFIG_RESERVED_1_MASK                                  (0xFFC0U)
#define CSL_ADC_ADCPPB3CONFIG_RESERVED_1_SHIFT                                 (0x0006U)
#define CSL_ADC_ADCPPB3CONFIG_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB3CONFIG_RESERVED_1_MAX                                   (0x03FFU)

#define CSL_ADC_ADCPPB3CONFIG_RESETVAL                                         (0x0000U)

/* ADCPPB3STAMP */

#define CSL_ADC_ADCPPB3STAMP_DLYSTAMP_MASK                                     (0x0FFFU)
#define CSL_ADC_ADCPPB3STAMP_DLYSTAMP_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCPPB3STAMP_DLYSTAMP_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCPPB3STAMP_DLYSTAMP_MAX                                      (0x0FFFU)

#define CSL_ADC_ADCPPB3STAMP_RESERVED_1_MASK                                   (0xF000U)
#define CSL_ADC_ADCPPB3STAMP_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCPPB3STAMP_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCPPB3STAMP_RESERVED_1_MAX                                    (0x000FU)

#define CSL_ADC_ADCPPB3STAMP_RESETVAL                                          (0x0000U)

/* ADCPPB3OFFCAL */

#define CSL_ADC_ADCPPB3OFFCAL_OFFCAL_MASK                                      (0x03FFU)
#define CSL_ADC_ADCPPB3OFFCAL_OFFCAL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB3OFFCAL_OFFCAL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB3OFFCAL_OFFCAL_MAX                                       (0x03FFU)

#define CSL_ADC_ADCPPB3OFFCAL_RESERVED_1_MASK                                  (0xFC00U)
#define CSL_ADC_ADCPPB3OFFCAL_RESERVED_1_SHIFT                                 (0x000AU)
#define CSL_ADC_ADCPPB3OFFCAL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB3OFFCAL_RESERVED_1_MAX                                   (0x003FU)

#define CSL_ADC_ADCPPB3OFFCAL_RESETVAL                                         (0x0000U)

/* ADCPPB3OFFREF */

#define CSL_ADC_ADCPPB3OFFREF_OFFREF_MASK                                      (0xFFFFU)
#define CSL_ADC_ADCPPB3OFFREF_OFFREF_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB3OFFREF_OFFREF_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB3OFFREF_OFFREF_MAX                                       (0xFFFFU)

#define CSL_ADC_ADCPPB3OFFREF_RESETVAL                                         (0x0000U)

/* ADCPPB3TRIPHI */

#define CSL_ADC_ADCPPB3TRIPHI_LIMITHI_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB3TRIPHI_LIMITHI_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPHI_LIMITHI_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPHI_LIMITHI_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB3TRIPHI_HSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB3TRIPHI_HSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB3TRIPHI_HSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPHI_HSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB3TRIPHI_RESERVED_1_MASK                                  (0xFFFE0000U)
#define CSL_ADC_ADCPPB3TRIPHI_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB3TRIPHI_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPHI_RESERVED_1_MAX                                   (0x00007FFFU)

#define CSL_ADC_ADCPPB3TRIPHI_RESETVAL                                         (0x00000000U)

/* ADCPPB3TRIPLO */

#define CSL_ADC_ADCPPB3TRIPLO_LIMITLO_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB3TRIPLO_LIMITLO_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPLO_LIMITLO_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPLO_LIMITLO_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB3TRIPLO_LSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB3TRIPLO_LSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB3TRIPLO_LSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPLO_LSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB3TRIPLO_RESERVED_1_MASK                                  (0x000E0000U)
#define CSL_ADC_ADCPPB3TRIPLO_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB3TRIPLO_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPLO_RESERVED_1_MAX                                   (0x00000007U)

#define CSL_ADC_ADCPPB3TRIPLO_REQSTAMP_MASK                                    (0xFFF00000U)
#define CSL_ADC_ADCPPB3TRIPLO_REQSTAMP_SHIFT                                   (0x00000014U)
#define CSL_ADC_ADCPPB3TRIPLO_REQSTAMP_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCPPB3TRIPLO_REQSTAMP_MAX                                     (0x00000FFFU)

#define CSL_ADC_ADCPPB3TRIPLO_RESETVAL                                         (0x00000000U)

/* ADCPPB4CONFIG */

#define CSL_ADC_ADCPPB4CONFIG_CONFIG_MASK                                      (0x000FU)
#define CSL_ADC_ADCPPB4CONFIG_CONFIG_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB4CONFIG_CONFIG_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB4CONFIG_CONFIG_MAX                                       (0x000FU)

#define CSL_ADC_ADCPPB4CONFIG_TWOSCOMPEN_MASK                                  (0x0010U)
#define CSL_ADC_ADCPPB4CONFIG_TWOSCOMPEN_SHIFT                                 (0x0004U)
#define CSL_ADC_ADCPPB4CONFIG_TWOSCOMPEN_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB4CONFIG_TWOSCOMPEN_MAX                                   (0x0001U)

#define CSL_ADC_ADCPPB4CONFIG_CBCEN_MASK                                       (0x0020U)
#define CSL_ADC_ADCPPB4CONFIG_CBCEN_SHIFT                                      (0x0005U)
#define CSL_ADC_ADCPPB4CONFIG_CBCEN_RESETVAL                                   (0x0000U)
#define CSL_ADC_ADCPPB4CONFIG_CBCEN_MAX                                        (0x0001U)

#define CSL_ADC_ADCPPB4CONFIG_RESERVED_1_MASK                                  (0xFFC0U)
#define CSL_ADC_ADCPPB4CONFIG_RESERVED_1_SHIFT                                 (0x0006U)
#define CSL_ADC_ADCPPB4CONFIG_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB4CONFIG_RESERVED_1_MAX                                   (0x03FFU)

#define CSL_ADC_ADCPPB4CONFIG_RESETVAL                                         (0x0000U)

/* ADCPPB4STAMP */

#define CSL_ADC_ADCPPB4STAMP_DLYSTAMP_MASK                                     (0x0FFFU)
#define CSL_ADC_ADCPPB4STAMP_DLYSTAMP_SHIFT                                    (0x0000U)
#define CSL_ADC_ADCPPB4STAMP_DLYSTAMP_RESETVAL                                 (0x0000U)
#define CSL_ADC_ADCPPB4STAMP_DLYSTAMP_MAX                                      (0x0FFFU)

#define CSL_ADC_ADCPPB4STAMP_RESERVED_1_MASK                                   (0xF000U)
#define CSL_ADC_ADCPPB4STAMP_RESERVED_1_SHIFT                                  (0x000CU)
#define CSL_ADC_ADCPPB4STAMP_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_ADC_ADCPPB4STAMP_RESERVED_1_MAX                                    (0x000FU)

#define CSL_ADC_ADCPPB4STAMP_RESETVAL                                          (0x0000U)

/* ADCPPB4OFFCAL */

#define CSL_ADC_ADCPPB4OFFCAL_OFFCAL_MASK                                      (0x03FFU)
#define CSL_ADC_ADCPPB4OFFCAL_OFFCAL_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB4OFFCAL_OFFCAL_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB4OFFCAL_OFFCAL_MAX                                       (0x03FFU)

#define CSL_ADC_ADCPPB4OFFCAL_RESERVED_1_MASK                                  (0xFC00U)
#define CSL_ADC_ADCPPB4OFFCAL_RESERVED_1_SHIFT                                 (0x000AU)
#define CSL_ADC_ADCPPB4OFFCAL_RESERVED_1_RESETVAL                              (0x0000U)
#define CSL_ADC_ADCPPB4OFFCAL_RESERVED_1_MAX                                   (0x003FU)

#define CSL_ADC_ADCPPB4OFFCAL_RESETVAL                                         (0x0000U)

/* ADCPPB4OFFREF */

#define CSL_ADC_ADCPPB4OFFREF_OFFREF_MASK                                      (0xFFFFU)
#define CSL_ADC_ADCPPB4OFFREF_OFFREF_SHIFT                                     (0x0000U)
#define CSL_ADC_ADCPPB4OFFREF_OFFREF_RESETVAL                                  (0x0000U)
#define CSL_ADC_ADCPPB4OFFREF_OFFREF_MAX                                       (0xFFFFU)

#define CSL_ADC_ADCPPB4OFFREF_RESETVAL                                         (0x0000U)

/* ADCPPB4TRIPHI */

#define CSL_ADC_ADCPPB4TRIPHI_LIMITHI_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB4TRIPHI_LIMITHI_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPHI_LIMITHI_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPHI_LIMITHI_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB4TRIPHI_HSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB4TRIPHI_HSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB4TRIPHI_HSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPHI_HSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB4TRIPHI_RESERVED_1_MASK                                  (0xFFFE0000U)
#define CSL_ADC_ADCPPB4TRIPHI_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB4TRIPHI_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPHI_RESERVED_1_MAX                                   (0x00007FFFU)

#define CSL_ADC_ADCPPB4TRIPHI_RESETVAL                                         (0x00000000U)

/* ADCPPB4TRIPLO */

#define CSL_ADC_ADCPPB4TRIPLO_LIMITLO_MASK                                     (0x0000FFFFU)
#define CSL_ADC_ADCPPB4TRIPLO_LIMITLO_SHIFT                                    (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPLO_LIMITLO_RESETVAL                                 (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPLO_LIMITLO_MAX                                      (0x0000FFFFU)

#define CSL_ADC_ADCPPB4TRIPLO_LSIGN_MASK                                       (0x00010000U)
#define CSL_ADC_ADCPPB4TRIPLO_LSIGN_SHIFT                                      (0x00000010U)
#define CSL_ADC_ADCPPB4TRIPLO_LSIGN_RESETVAL                                   (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPLO_LSIGN_MAX                                        (0x00000001U)

#define CSL_ADC_ADCPPB4TRIPLO_RESERVED_1_MASK                                  (0x000E0000U)
#define CSL_ADC_ADCPPB4TRIPLO_RESERVED_1_SHIFT                                 (0x00000011U)
#define CSL_ADC_ADCPPB4TRIPLO_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPLO_RESERVED_1_MAX                                   (0x00000007U)

#define CSL_ADC_ADCPPB4TRIPLO_REQSTAMP_MASK                                    (0xFFF00000U)
#define CSL_ADC_ADCPPB4TRIPLO_REQSTAMP_SHIFT                                   (0x00000014U)
#define CSL_ADC_ADCPPB4TRIPLO_REQSTAMP_RESETVAL                                (0x00000000U)
#define CSL_ADC_ADCPPB4TRIPLO_REQSTAMP_MAX                                     (0x00000FFFU)

#define CSL_ADC_ADCPPB4TRIPLO_RESETVAL                                         (0x00000000U)

/* ADCINTCYCLE */

#define CSL_ADC_ADCINTCYCLE_DELAY_MASK                                         (0xFFFFU)
#define CSL_ADC_ADCINTCYCLE_DELAY_SHIFT                                        (0x0000U)
#define CSL_ADC_ADCINTCYCLE_DELAY_RESETVAL                                     (0x0000U)
#define CSL_ADC_ADCINTCYCLE_DELAY_MAX                                          (0xFFFFU)

#define CSL_ADC_ADCINTCYCLE_RESETVAL                                           (0x0000U)

/* ADCINLTRIM1 */

#define CSL_ADC_ADCINLTRIM1_INLTRIM31TO0_MASK                                  (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM1_INLTRIM31TO0_SHIFT                                 (0x00000000U)
#define CSL_ADC_ADCINLTRIM1_INLTRIM31TO0_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCINLTRIM1_INLTRIM31TO0_MAX                                   (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM1_RESETVAL                                           (0x00000000U)

/* ADCINLTRIM2 */

#define CSL_ADC_ADCINLTRIM2_INLTRIM63TO32_MASK                                 (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM2_INLTRIM63TO32_SHIFT                                (0x00000000U)
#define CSL_ADC_ADCINLTRIM2_INLTRIM63TO32_RESETVAL                             (0x00000000U)
#define CSL_ADC_ADCINLTRIM2_INLTRIM63TO32_MAX                                  (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM2_RESETVAL                                           (0x00000000U)

/* ADCINLTRIM3 */

#define CSL_ADC_ADCINLTRIM3_INLTRIM95TO64_MASK                                 (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM3_INLTRIM95TO64_SHIFT                                (0x00000000U)
#define CSL_ADC_ADCINLTRIM3_INLTRIM95TO64_RESETVAL                             (0x00000000U)
#define CSL_ADC_ADCINLTRIM3_INLTRIM95TO64_MAX                                  (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM3_RESETVAL                                           (0x00000000U)

/* ADCINLTRIM4 */

#define CSL_ADC_ADCINLTRIM4_INLTRIM127TO96_MASK                                (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM4_INLTRIM127TO96_SHIFT                               (0x00000000U)
#define CSL_ADC_ADCINLTRIM4_INLTRIM127TO96_RESETVAL                            (0x00000000U)
#define CSL_ADC_ADCINLTRIM4_INLTRIM127TO96_MAX                                 (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM4_RESETVAL                                           (0x00000000U)

/* ADCINLTRIM5 */

#define CSL_ADC_ADCINLTRIM5_INLTRIM159TO128_MASK                               (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM5_INLTRIM159TO128_SHIFT                              (0x00000000U)
#define CSL_ADC_ADCINLTRIM5_INLTRIM159TO128_RESETVAL                           (0x00000000U)
#define CSL_ADC_ADCINLTRIM5_INLTRIM159TO128_MAX                                (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM5_RESETVAL                                           (0x00000000U)

/* ADCINLTRIM6 */

#define CSL_ADC_ADCINLTRIM6_INLTRIM191TO160_MASK                               (0xFFFFFFFFU)
#define CSL_ADC_ADCINLTRIM6_INLTRIM191TO160_SHIFT                              (0x00000000U)
#define CSL_ADC_ADCINLTRIM6_INLTRIM191TO160_RESETVAL                           (0x00000000U)
#define CSL_ADC_ADCINLTRIM6_INLTRIM191TO160_MAX                                (0xFFFFFFFFU)

#define CSL_ADC_ADCINLTRIM6_RESETVAL                                           (0x00000000U)

/* ADCINLTRIMCTL */

#define CSL_ADC_ADCINLTRIMCTL_CALIBMODE_MASK                                   (0x00000001U)
#define CSL_ADC_ADCINLTRIMCTL_CALIBMODE_SHIFT                                  (0x00000000U)
#define CSL_ADC_ADCINLTRIMCTL_CALIBMODE_RESETVAL                               (0x00000000U)
#define CSL_ADC_ADCINLTRIMCTL_CALIBMODE_MAX                                    (0x00000001U)

#define CSL_ADC_ADCINLTRIMCTL_CALIBSTEP_MASK                                   (0x0000003EU)
#define CSL_ADC_ADCINLTRIMCTL_CALIBSTEP_SHIFT                                  (0x00000001U)
#define CSL_ADC_ADCINLTRIMCTL_CALIBSTEP_RESETVAL                               (0x00000000U)
#define CSL_ADC_ADCINLTRIMCTL_CALIBSTEP_MAX                                    (0x0000001FU)

#define CSL_ADC_ADCINLTRIMCTL_RESERVED_1_MASK                                  (0x0000FFC0U)
#define CSL_ADC_ADCINLTRIMCTL_RESERVED_1_SHIFT                                 (0x00000006U)
#define CSL_ADC_ADCINLTRIMCTL_RESERVED_1_RESETVAL                              (0x00000000U)
#define CSL_ADC_ADCINLTRIMCTL_RESERVED_1_MAX                                   (0x000003FFU)

#define CSL_ADC_ADCINLTRIMCTL_KEY_MASK                                         (0xFFFF0000U)
#define CSL_ADC_ADCINLTRIMCTL_KEY_SHIFT                                        (0x00000010U)
#define CSL_ADC_ADCINLTRIMCTL_KEY_RESETVAL                                     (0x00000000U)
#define CSL_ADC_ADCINLTRIMCTL_KEY_MAX                                          (0x0000FFFFU)

#define CSL_ADC_ADCINLTRIMCTL_RESETVAL                                         (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
