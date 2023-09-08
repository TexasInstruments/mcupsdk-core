/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_resolver.h
*/
#ifndef CSLR_RESOLVER_H_
#define CSLR_RESOLVER_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_RESOLVER_REGS_REGS_BASE                                            (0x00000000U)


/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVID;                     /* Module ID and Version  */
    volatile uint32_t GLOBAL_CFG;                /* Global Configuration */
    volatile uint32_t EXCIT_SAMPLE_CFG1;         /* Excitation and Sample Configuration1 */
    volatile uint32_t EXCIT_SAMPLE_CFG2;         /* Excitation and Sample Configuration2 */
    volatile uint32_t EXCIT_SAMPLE_CFG3;         /* Excitation and Sample Configuration3 */
    volatile uint32_t IRQSTATUS_RAW_SYS;         /* Interrupt Raw Status  */
    volatile uint32_t IRQSTATUS_SYS;             /* Interrupt Status  */
    volatile uint32_t IRQENABLE_SET_SYS;         /* Interrupt Enable Set  */
    volatile uint32_t IRQENABLE_CLR_SYS;         /* Interrupt Enable Clear  */
    volatile uint32_t CAL_CFG;                   /* CAL Config */
    volatile uint32_t CAL_OBS;                   /* CAL ADC0 and ADC1 Data */
    volatile uint8_t  Resv_48[4];
    volatile uint32_t IRQSTATUS_RAW_SYS_0;       /* Interrupt Raw Status  */
    volatile uint32_t IRQSTATUS_SYS_0;           /* Interrupt Status  */
    volatile uint32_t IRQENABLE_SET_SYS_0;       /* Interrupt Enable Set  */
    volatile uint32_t IRQENABLE_CLR_SYS_0;       /* Interrupt Enable Clear  */
    volatile uint32_t DC_OFF_CFG1_0;             /* DC Offset Config1 */
    volatile uint32_t DC_OFF_CFG2_0;             /* DC Offset Config2 */
    volatile uint32_t DC_OFF0;                   /* DC Offset */
    volatile uint32_t SAMPLE_CFG1_0;             /* Ideal Sample Config */
    volatile uint32_t SAMPLE_CFG2_0;             /* Ideal Sample Config */
    volatile uint32_t DEC_GF_CFG0;               /* Dec GF Config */
    volatile uint32_t PG_EST_CFG1_0;             /* Phase and Gain Est Config1 */
    volatile uint32_t PG_EST_CFG2_0;             /* Phase and Gain Est Config2 */
    volatile uint32_t PG_EST_CFG3_0;             /* Phase and Gain Est Config3 */
    volatile uint32_t PG_EST_CFG4_0;             /* Phase and Gain Est Config4 */
    volatile uint32_t PG_EST_CFG5_0;             /* Phase and Gain Est Config4 */
    volatile uint32_t PG_EST_CFG6_0;             /* Phase and Gain Est Config4 */
    volatile uint32_t TRACK2_CFG1_0;             /* Kp, Ki, Kd, Kvelfilt, Kffw, Boost Cfg1 */
    volatile uint32_t TRACK2_CFG2_0;             /* Feed fwd Boost Kffw Cfg2 */
    volatile uint32_t TRACK2_CFG3_0;             /* Cfg3 */
    volatile uint32_t ANGLE_ARCTAN_0;            /* Angle and Velocity from arctan */
    volatile uint32_t ANGLE_TRACK2_0;            /* Angle and Velocity from track2 */
    volatile uint32_t VELOCITY_TRACK2_0;         /* Angle and Velocity from track2 */
    volatile uint32_t OFFSET_0;                  /* SIN and COS Offset */
    volatile uint32_t TRACK_ERR_0;               /* Tracking error Cfg */
    volatile uint32_t TRACK_ERR2_0;              /* Tracking error Cfg2 */
    volatile uint32_t DIAG0_0;                   /* Diag0 0 */
    volatile uint32_t DIAG1_0;                   /* Diag1 0 */
    volatile uint32_t DIAG2_0;                   /* Diag2 0 */
    volatile uint32_t DIAG3_0;                   /* Diag3 0 */
    volatile uint32_t DIAG4_0;                   /* Diag4 0 */
    volatile uint32_t DIAG5_0;                   /* Diag5 0 */
    volatile uint32_t DIAG6_0;                   /* Diag6 0 */
    volatile uint32_t DIAG7_0;                   /* Diag7 0 */
    volatile uint32_t DIAG8_0;                   /* Diag8 0 */
    volatile uint32_t DIAG9_0;                   /* Diag9 0 */
    volatile uint32_t DIAG10_0;                  /* Diag10 0 */
    volatile uint32_t DIAG11_0;                  /* Diag11 0 */
    volatile uint32_t DIAG12_0;                  /* Diag12 0 */
    volatile uint32_t DIAG13_0;                  /* Diag13 0 */
    volatile uint32_t DIAG14_0;                  /* Diag14 0 */
    volatile uint32_t DIAG15_0;                  /* Diag15 0 */
    volatile uint32_t DIAG16_0;                  /* Diag16 0 */
    volatile uint32_t DIAG17_0;                  /* Diag17 0 */
    volatile uint32_t DIAG18_0;                  /* Diag18 0 */
    volatile uint32_t DIAG19_0;                  /* Diag19 0 */
    volatile uint32_t DIAG20_0;                  /* Diag20 0 */
    volatile uint32_t DIAG21_0;                  /* Diag21 0 */
    volatile uint32_t OBS_ADC_0;                 /* Sw Obs adc data post latch and avg if enabled  */
    volatile uint32_t OBS_ADC_REC_0;             /* Sw Obs adc data post recovered data, so post dec */
    volatile uint32_t OBS_ADC_DC_0;              /* Sw Obs adc data post dc offset */
    volatile uint32_t OBS_ADC_PGC_0;             /* Sw Obs post phase and gain correction  */
    volatile uint32_t OBS_NOISE_VAL_0;           /* Sw Obs last cos noise value over noise_threshold  */
    volatile uint32_t OBS_PEAKHISTOGRAM3_0_0;    /* SW obs peak histogram buckets */
    volatile uint32_t OBS_PEAKHISTOGRAM7_4_0;    /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM11_8_0;   /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM15_12_0;   /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM19_16_0;   /* SW obs peak histogram buckets  */
    volatile uint8_t  Resv_512[236];
    volatile uint32_t IRQSTATUS_RAW_SYS_1;       /* Interrupt Raw Status  */
    volatile uint32_t IRQSTATUS_SYS_1;           /* Interrupt Status  */
    volatile uint32_t IRQENABLE_SET_SYS_1;       /* Interrupt Enable Set  */
    volatile uint32_t IRQENABLE_CLR_SYS_1;       /* Interrupt Enable Clear  */
    volatile uint32_t DC_OFF_CFG1_1;             /* DC Offset Config1 */
    volatile uint32_t DC_OFF_CFG2_1;             /* DC Offset Config2 */
    volatile uint32_t DC_OFF1;                   /* DC Offset */
    volatile uint32_t SAMPLE_CFG1_1;             /* Ideal Sample Config */
    volatile uint32_t SAMPLE_CFG2_1;             /* Ideal Sample Config */
    volatile uint32_t DEC_GF_CFG1;               /* Dec GF Config */
    volatile uint32_t PG_EST_CFG1_1;             /* Phase and Gain Est Config1 */
    volatile uint32_t PG_EST_CFG2_1;             /* Phase and Gain Est Config2 */
    volatile uint32_t PG_EST_CFG3_1;             /* Phase and Gain Est Config3 */
    volatile uint32_t PG_EST_CFG4_1;             /* Phase and Gain Est Config4 */
    volatile uint32_t PG_EST_CFG5_1;             /* Phase and Gain Est Config4 */
    volatile uint32_t PG_EST_CFG6_1;             /* Phase and Gain Est Config4 */
    volatile uint32_t TRACK2_CFG1_1;             /* Kp, Ki, Kd, Kvelfilt, Kffw, Boost Cfg1 */
    volatile uint32_t TRACK2_CFG2_1;             /* Feed fwd Boost Kffw Cfg2 */
    volatile uint32_t TRACK2_CFG3_1;             /* Cfg3 */
    volatile uint32_t ANGLE_ARCTAN_1;            /* Angle and Velocity from arctan */
    volatile uint32_t ANGLE_TRACK2_1;            /* Angle and Velocity from track2 */
    volatile uint32_t VELOCITY_TRACK2_1;         /* Angle and Velocity from track2 */
    volatile uint32_t OFFSET_1;                  /* SIN and COS Offset */
    volatile uint32_t TRACK_ERR_1;               /* Tracking error Cfg */
    volatile uint32_t TRACK_ERR2_1;              /* Tracking error Cfg2 */
    volatile uint32_t DIAG0_1;                   /* Diag0 1 */
    volatile uint32_t DIAG1_1;                   /* Diag1 1 */
    volatile uint32_t DIAG2_1;                   /* Diag2 1 */
    volatile uint32_t DIAG3_1;                   /* Diag3 1 */
    volatile uint32_t DIAG4_1;                   /* Diag4 1 */
    volatile uint32_t DIAG5_1;                   /* Diag5 1 */
    volatile uint32_t DIAG6_1;                   /* Diag6 1 */
    volatile uint32_t DIAG7_1;                   /* Diag7 1 */
    volatile uint32_t DIAG8_1;                   /* Diag8 1 */
    volatile uint32_t DIAG9_1;                   /* Diag9 1 */
    volatile uint32_t DIAG10_1;                  /* Diag10 1 */
    volatile uint32_t DIAG11_1;                  /* Diag11 1 */
    volatile uint32_t DIAG12_1;                  /* Diag12 1 */
    volatile uint32_t DIAG13_1;                  /* Diag13 1 */
    volatile uint32_t DIAG14_1;                  /* Diag14 1 */
    volatile uint32_t DIAG15_1;                  /* Diag15 1 */
    volatile uint32_t DIAG16_1;                  /* Diag16 1 */
    volatile uint32_t DIAG17_1;                  /* Diag17 1 */
    volatile uint32_t DIAG18_1;                  /* Diag18 1 */
    volatile uint32_t DIAG19_1;                  /* Diag19 1 */
    volatile uint32_t DIAG20_1;                  /* Diag20 1 */
    volatile uint32_t DIAG21_1;                  /* Diag21 1 */
    volatile uint32_t OBS_ADC_1;                 /* Sw Obs adc data post latch and avg if enabled  */
    volatile uint32_t OBS_ADC_REC_1;             /* Sw Obs adc data post recovered data, so post dec */
    volatile uint32_t OBS_ADC_DC_1;              /* Sw Obs adc data post dc offset */
    volatile uint32_t OBS_ADC_PGC_1;             /* Sw Obs post phase and gain correction  */
    volatile uint32_t OBS_NOISE_VAL_1;           /* Sw Obs last cos noise value over noise_threshold  */
    volatile uint32_t OBS_PEAKHISTOGRAM3_0_1;    /* SW obs peak histogram buckets */
    volatile uint32_t OBS_PEAKHISTOGRAM7_4_1;    /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM11_8_1;   /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM15_12_1;   /* SW obs peak histogram buckets  */
    volatile uint32_t OBS_PEAKHISTOGRAM19_16_1;   /* SW obs peak histogram buckets  */
} CSL_resolver_regsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RESOLVER_REGS_REVID                                                (0x00000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG                                           (0x00000004U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1                                    (0x00000008U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2                                    (0x0000000CU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3                                    (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS                                    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS                                        (0x00000018U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS                                    (0x0000001CU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS                                    (0x00000020U)
#define CSL_RESOLVER_REGS_CAL_CFG                                              (0x00000024U)
#define CSL_RESOLVER_REGS_CAL_OBS                                              (0x00000028U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0                                  (0x00000030U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0                                      (0x00000034U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0                                  (0x00000038U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0                                  (0x0000003CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0                                        (0x00000040U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0                                        (0x00000044U)
#define CSL_RESOLVER_REGS_DC_OFF0                                              (0x00000048U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0                                        (0x0000004CU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0                                        (0x00000050U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0                                          (0x00000054U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0                                        (0x00000058U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0                                        (0x0000005CU)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0                                        (0x00000060U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0                                        (0x00000064U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_0                                        (0x00000068U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_0                                        (0x0000006CU)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0                                        (0x00000070U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0                                        (0x00000074U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0                                        (0x00000078U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0                                       (0x0000007CU)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0                                       (0x00000080U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0                                    (0x00000084U)
#define CSL_RESOLVER_REGS_OFFSET_0                                             (0x00000088U)
#define CSL_RESOLVER_REGS_TRACK_ERR_0                                          (0x0000008CU)
#define CSL_RESOLVER_REGS_TRACK_ERR2_0                                         (0x00000090U)
#define CSL_RESOLVER_REGS_DIAG0_0                                              (0x00000094U)
#define CSL_RESOLVER_REGS_DIAG1_0                                              (0x00000098U)
#define CSL_RESOLVER_REGS_DIAG2_0                                              (0x0000009CU)
#define CSL_RESOLVER_REGS_DIAG3_0                                              (0x000000A0U)
#define CSL_RESOLVER_REGS_DIAG4_0                                              (0x000000A4U)
#define CSL_RESOLVER_REGS_DIAG5_0                                              (0x000000A8U)
#define CSL_RESOLVER_REGS_DIAG6_0                                              (0x000000ACU)
#define CSL_RESOLVER_REGS_DIAG7_0                                              (0x000000B0U)
#define CSL_RESOLVER_REGS_DIAG8_0                                              (0x000000B4U)
#define CSL_RESOLVER_REGS_DIAG9_0                                              (0x000000B8U)
#define CSL_RESOLVER_REGS_DIAG10_0                                             (0x000000BCU)
#define CSL_RESOLVER_REGS_DIAG11_0                                             (0x000000C0U)
#define CSL_RESOLVER_REGS_DIAG12_0                                             (0x000000C4U)
#define CSL_RESOLVER_REGS_DIAG13_0                                             (0x000000C8U)
#define CSL_RESOLVER_REGS_DIAG14_0                                             (0x000000CCU)
#define CSL_RESOLVER_REGS_DIAG15_0                                             (0x000000D0U)
#define CSL_RESOLVER_REGS_DIAG16_0                                             (0x000000D4U)
#define CSL_RESOLVER_REGS_DIAG17_0                                             (0x000000D8U)
#define CSL_RESOLVER_REGS_DIAG18_0                                             (0x000000DCU)
#define CSL_RESOLVER_REGS_DIAG19_0                                             (0x000000E0U)
#define CSL_RESOLVER_REGS_DIAG20_0                                             (0x000000E4U)
#define CSL_RESOLVER_REGS_DIAG21_0                                             (0x000000E8U)
#define CSL_RESOLVER_REGS_OBS_ADC_0                                            (0x000000ECU)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0                                        (0x000000F0U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0                                         (0x000000F4U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0                                        (0x000000F8U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0                                      (0x000000FCU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0                               (0x00000100U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0                               (0x00000104U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0                              (0x00000108U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0                             (0x0000010CU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0                             (0x00000110U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1                                  (0x00000200U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1                                      (0x00000204U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1                                  (0x00000208U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1                                  (0x0000020CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1                                        (0x00000210U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1                                        (0x00000214U)
#define CSL_RESOLVER_REGS_DC_OFF1                                              (0x00000218U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1                                        (0x0000021CU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1                                        (0x00000220U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1                                          (0x00000224U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1                                        (0x00000228U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1                                        (0x0000022CU)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1                                        (0x00000230U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1                                        (0x00000234U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_1                                        (0x00000238U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_1                                        (0x0000023CU)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1                                        (0x00000240U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1                                        (0x00000244U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1                                        (0x00000248U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1                                       (0x0000024CU)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1                                       (0x00000250U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1                                    (0x00000254U)
#define CSL_RESOLVER_REGS_OFFSET_1                                             (0x00000258U)
#define CSL_RESOLVER_REGS_TRACK_ERR_1                                          (0x0000025CU)
#define CSL_RESOLVER_REGS_TRACK_ERR2_1                                         (0x00000260U)
#define CSL_RESOLVER_REGS_DIAG0_1                                              (0x00000264U)
#define CSL_RESOLVER_REGS_DIAG1_1                                              (0x00000268U)
#define CSL_RESOLVER_REGS_DIAG2_1                                              (0x0000026CU)
#define CSL_RESOLVER_REGS_DIAG3_1                                              (0x00000270U)
#define CSL_RESOLVER_REGS_DIAG4_1                                              (0x00000274U)
#define CSL_RESOLVER_REGS_DIAG5_1                                              (0x00000278U)
#define CSL_RESOLVER_REGS_DIAG6_1                                              (0x0000027CU)
#define CSL_RESOLVER_REGS_DIAG7_1                                              (0x00000280U)
#define CSL_RESOLVER_REGS_DIAG8_1                                              (0x00000284U)
#define CSL_RESOLVER_REGS_DIAG9_1                                              (0x00000288U)
#define CSL_RESOLVER_REGS_DIAG10_1                                             (0x0000028CU)
#define CSL_RESOLVER_REGS_DIAG11_1                                             (0x00000290U)
#define CSL_RESOLVER_REGS_DIAG12_1                                             (0x00000294U)
#define CSL_RESOLVER_REGS_DIAG13_1                                             (0x00000298U)
#define CSL_RESOLVER_REGS_DIAG14_1                                             (0x0000029CU)
#define CSL_RESOLVER_REGS_DIAG15_1                                             (0x000002A0U)
#define CSL_RESOLVER_REGS_DIAG16_1                                             (0x000002A4U)
#define CSL_RESOLVER_REGS_DIAG17_1                                             (0x000002A8U)
#define CSL_RESOLVER_REGS_DIAG18_1                                             (0x000002ACU)
#define CSL_RESOLVER_REGS_DIAG19_1                                             (0x000002B0U)
#define CSL_RESOLVER_REGS_DIAG20_1                                             (0x000002B4U)
#define CSL_RESOLVER_REGS_DIAG21_1                                             (0x000002B8U)
#define CSL_RESOLVER_REGS_OBS_ADC_1                                            (0x000002BCU)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1                                        (0x000002C0U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1                                         (0x000002C4U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1                                        (0x000002C8U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1                                      (0x000002CCU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1                               (0x000002D0U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1                               (0x000002D4U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1                              (0x000002D8U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1                             (0x000002DCU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1                             (0x000002E0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVID */

#define CSL_RESOLVER_REGS_REVID_REG_MODULE_ID_MASK                             (0xFFFF0000U)
#define CSL_RESOLVER_REGS_REVID_REG_MODULE_ID_SHIFT                            (0x00000010U)
#define CSL_RESOLVER_REGS_REVID_REG_MODULE_ID_RESETVAL                         (0x000068CFU)
#define CSL_RESOLVER_REGS_REVID_REG_MODULE_ID_MAX                              (0x0000FFFFU)

#define CSL_RESOLVER_REGS_REVID_REG_RTL_VERSION_MASK                           (0x0000F800U)
#define CSL_RESOLVER_REGS_REVID_REG_RTL_VERSION_SHIFT                          (0x0000000BU)
#define CSL_RESOLVER_REGS_REVID_REG_RTL_VERSION_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_REVID_REG_RTL_VERSION_MAX                            (0x0000001FU)

#define CSL_RESOLVER_REGS_REVID_REG_MAJOR_REVISION_MASK                        (0x00000700U)
#define CSL_RESOLVER_REGS_REVID_REG_MAJOR_REVISION_SHIFT                       (0x00000008U)
#define CSL_RESOLVER_REGS_REVID_REG_MAJOR_REVISION_RESETVAL                    (0x00000001U)
#define CSL_RESOLVER_REGS_REVID_REG_MAJOR_REVISION_MAX                         (0x00000007U)

#define CSL_RESOLVER_REGS_REVID_REG_CUSTOM_REVISION_MASK                       (0x000000C0U)
#define CSL_RESOLVER_REGS_REVID_REG_CUSTOM_REVISION_SHIFT                      (0x00000006U)
#define CSL_RESOLVER_REGS_REVID_REG_CUSTOM_REVISION_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_REVID_REG_CUSTOM_REVISION_MAX                        (0x00000003U)

#define CSL_RESOLVER_REGS_REVID_REG_MINOR_REVISION_MASK                        (0x0000003FU)
#define CSL_RESOLVER_REGS_REVID_REG_MINOR_REVISION_SHIFT                       (0x00000000U)
#define CSL_RESOLVER_REGS_REVID_REG_MINOR_REVISION_RESETVAL                    (0x00000000U)
#define CSL_RESOLVER_REGS_REVID_REG_MINOR_REVISION_MAX                         (0x0000003FU)

#define CSL_RESOLVER_REGS_REVID_RESETVAL                                       (0x68CF0100U)

/* GLOBAL_CFG */

#define CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MASK                            (0x00000001U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_SHIFT                           (0x00000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MASTER_EN_MAX                             (0x00000001U)

#define CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_MASK                                 (0x00000F00U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_SHIFT                                (0x00000008U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_RESETVAL                             (0x00000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_MODE_MAX                                  (0x0000000FU)

#define CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MASK                            (0x00001000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_SHIFT                           (0x0000000CU)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_RESETVAL                        (0x00000001U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SINGLE_EN_MAX                             (0x00000001U)

#define CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_MASK                            (0x00FF0000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_SHIFT                           (0x00000010U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_RESETVAL                        (0x00000001U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_BURST_CNT_MAX                             (0x000000FFU)

#define CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_MASK                            (0xFF000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_SHIFT                           (0x00000018U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_GLOBAL_CFG_SOC_WIDTH_MAX                             (0x000000FFU)

#define CSL_RESOLVER_REGS_GLOBAL_CFG_RESETVAL                                  (0x00011000U)

/* EXCIT_SAMPLE_CFG1 */

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_MASK                  (0x000000FFU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_RESETVAL              (0x000000C8U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_SEL_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_MASK               (0x0000FF00U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_SHIFT              (0x00000008U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_RESETVAL           (0x00000008U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_ADC_SAMPLE_RATE_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_MASK            (0x1FFF0000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_SHIFT           (0x00000010U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_RESETVAL        (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_EXC_FREQ_PHASE_CFG_MAX             (0x00001FFFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG1_RESETVAL                           (0x000008C8U)

/* EXCIT_SAMPLE_CFG2 */

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_MASK                (0x00001FFFU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_PHASE_INFO_MAX                 (0x00001FFFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_MASK          (0x1FFF0000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_SHIFT         (0x00000010U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_RESETVAL      (0x00000190U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_TO_SOC_DLY_START_MAX           (0x00001FFFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MASK                (0x20000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_SHIFT               (0x0000001DU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EN_MAX                 (0x00000001U)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_MASK             (0x80000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_SHIFT            (0x0000001FU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_PWM_SYNC_IN_EVENT_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG2_RESETVAL                           (0x01900000U)

/* EXCIT_SAMPLE_CFG3 */

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_MASK                  (0x000000FFU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_RESETVAL              (0x000000EFU)
#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_EXC_AMP_CTRL_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_EXCIT_SAMPLE_CFG3_RESETVAL                           (0x000000EFU)

/* IRQSTATUS_RAW_SYS */

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_MASK                       (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_SEQ_ERR_MAX                        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_RESETVAL                           (0x00000000U)

/* IRQSTATUS_SYS */

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MASK                           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_SEQ_ERR_MAX                            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_RESETVAL                               (0x00000000U)

/* IRQENABLE_SET_SYS */

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_SEQ_ERR_MASK                       (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_SEQ_ERR_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_SEQ_ERR_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_SEQ_ERR_MAX                        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_RESETVAL                           (0x00000000U)

/* IRQENABLE_CLR_SYS */

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_MASK                       (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_SEQ_ERR_MAX                        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_RESETVAL                           (0x00000000U)

/* CAL_CFG */

#define CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_MASK                                  (0x00000001U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_SHIFT                                 (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_RESETVAL                              (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_EN_MAX                                   (0x00000001U)

#define CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_MASK                               (0x000000F0U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_SHIFT                              (0x00000004U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_RESETVAL                           (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_CHSEL_MAX                                (0x0000000FU)

#define CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_MASK                                (0x00000100U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_SHIFT                               (0x00000008U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_RESETVAL                            (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_CFG_CAL_DONE_MAX                                 (0x00000001U)

#define CSL_RESOLVER_REGS_CAL_CFG_RESETVAL                                     (0x00000000U)

/* CAL_OBS */

#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC0_DATA_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_MASK                           (0xFFFF0000U)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_SHIFT                          (0x00000010U)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_CAL_OBS_CAL_ADC1_DATA_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_CAL_OBS_RESETVAL                                     (0x00000000U)

/* IRQSTATUS_RAW_SYS_0 */

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_LOWAMPLITUDE_ERR0_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_LOWAMPLITUDE_ERR0_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_LOWAMPLITUDE_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_LOWAMPLITUDE_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_LO_ERR0_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_LO_ERR0_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_LO_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_LO_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_HI_ERR0_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_HI_ERR0_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_HI_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SINSQCOSSQ_HI_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_HI_ERR0_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_HI_ERR0_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_HI_ERR0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_EXCFREQDRIFT_HI_ERR0_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_LO_ERR0_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_LO_ERR0_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_LO_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_LO_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_HI_ERR0_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_HI_ERR0_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_HI_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_PHASEDRIFT_COS_HI_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_LO_ERR0_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_LO_ERR0_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_HI_ERR0_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_HI_ERR0_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_SIN_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_LO_ERR0_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_LO_ERR0_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_HI_ERR0_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_HI_ERR0_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_GAINDRIFT_COS_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_LO_ERR0_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_HI_ERR0_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_TRACK_LOCK_ERR0_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_TRACK_LOCK_ERR0_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_TRACK_LOCK_ERR0_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_TRACK_LOCK_ERR0_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_0_RESETVAL                         (0x00000000U)

/* IRQSTATUS_SYS_0 */

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_LOWAMPLITUDE_ERR0_MASK               (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_LOWAMPLITUDE_ERR0_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_LOWAMPLITUDE_ERR0_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_LOWAMPLITUDE_ERR0_MAX                (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MASK    (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_SHIFT   (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MASK    (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_SHIFT   (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_LO_ERR0_MASK              (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_LO_ERR0_SHIFT             (0x00000003U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_LO_ERR0_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_LO_ERR0_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_HI_ERR0_MASK              (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_HI_ERR0_SHIFT             (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_HI_ERR0_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SINSQCOSSQ_HI_ERR0_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MASK         (0x00000020U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_MULTI_ZC_ERROR_ERR0_SHIFT        (0x00000005U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_MULTI_ZC_ERROR_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MASK         (0x00000040U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_SHIFT        (0x00000006U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MASK   (0x00000080U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT  (0x00000007U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MASK   (0x00000100U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT  (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MASK   (0x00000200U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT  (0x00000009U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MASK   (0x00000400U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT  (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MASK        (0x00000800U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_SHIFT       (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MASK        (0x00001000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_SHIFT       (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_HI_ERR0_MASK            (0x00002000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_HI_ERR0_SHIFT           (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_HI_ERR0_RESETVAL        (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_EXCFREQDRIFT_HI_ERR0_MAX             (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_LO_ERR0_MASK          (0x00004000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_LO_ERR0_SHIFT         (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_LO_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_LO_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_HI_ERR0_MASK          (0x00008000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_HI_ERR0_SHIFT         (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_HI_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_PHASEDRIFT_COS_HI_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_LO_ERR0_MASK           (0x00010000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_LO_ERR0_SHIFT          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_LO_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_LO_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_HI_ERR0_MASK           (0x00020000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_HI_ERR0_SHIFT          (0x00000011U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_HI_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_SIN_HI_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_LO_ERR0_MASK           (0x00040000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_LO_ERR0_SHIFT          (0x00000012U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_LO_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_LO_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_HI_ERR0_MASK           (0x00080000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_HI_ERR0_SHIFT          (0x00000013U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_HI_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_GAINDRIFT_COS_HI_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MASK         (0x00100000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_SHIFT        (0x00000014U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MASK         (0x00200000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_SHIFT        (0x00000015U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MASK         (0x00400000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_LO_ERR0_SHIFT        (0x00000016U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_LO_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MASK         (0x00800000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_HI_ERR0_SHIFT        (0x00000017U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_HI_ERR0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_TRACK_LOCK_ERR0_MASK                 (0x01000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_TRACK_LOCK_ERR0_SHIFT                (0x00000018U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_TRACK_LOCK_ERR0_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_TRACK_LOCK_ERR0_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_0_RESETVAL                             (0x00000000U)

/* IRQENABLE_SET_SYS_0 */

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_LOWAMPLITUDE_ERR0_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_LOWAMPLITUDE_ERR0_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_LOWAMPLITUDE_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_LOWAMPLITUDE_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_LO_ERR0_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_LO_ERR0_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_LO_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_LO_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_HI_ERR0_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_HI_ERR0_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_HI_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SINSQCOSSQ_HI_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_HI_ERR0_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_HI_ERR0_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_HI_ERR0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_EXCFREQDRIFT_HI_ERR0_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_LO_ERR0_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_LO_ERR0_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_LO_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_LO_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_HI_ERR0_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_HI_ERR0_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_HI_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_PHASEDRIFT_COS_HI_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_LO_ERR0_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_LO_ERR0_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_HI_ERR0_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_HI_ERR0_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_SIN_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_LO_ERR0_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_LO_ERR0_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_HI_ERR0_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_HI_ERR0_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_GAINDRIFT_COS_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_LO_ERR0_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_HI_ERR0_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_TRACK_LOCK_ERR0_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_TRACK_LOCK_ERR0_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_TRACK_LOCK_ERR0_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_TRACK_LOCK_ERR0_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_0_RESETVAL                         (0x00000000U)

/* IRQENABLE_CLR_SYS_0 */

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_LOWAMPLITUDE_ERR0_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_LOWAMPLITUDE_ERR0_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_LOWAMPLITUDE_ERR0_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_LOWAMPLITUDE_ERR0_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_COS_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_HIGHAMPLITUDE_SIN_FAULT_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_LO_ERR0_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_LO_ERR0_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_LO_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_LO_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_HI_ERR0_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_HI_ERR0_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_HI_ERR0_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SINSQCOSSQ_HI_ERR0_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_MULTI_ZC_ERROR_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_COS_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_SIN_POS_ZC_PEAK_MISMATCH_ERR0_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_SIN_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_COS_LO_ERR0_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_HI_ERR0_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_HI_ERR0_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_HI_ERR0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_EXCFREQDRIFT_HI_ERR0_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_LO_ERR0_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_LO_ERR0_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_LO_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_LO_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_HI_ERR0_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_HI_ERR0_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_HI_ERR0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_PHASEDRIFT_COS_HI_ERR0_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_LO_ERR0_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_LO_ERR0_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_HI_ERR0_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_HI_ERR0_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_SIN_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_LO_ERR0_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_LO_ERR0_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_LO_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_LO_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_HI_ERR0_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_HI_ERR0_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_HI_ERR0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_GAINDRIFT_COS_HI_ERR0_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_SIN_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_LO_ERR0_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_LO_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_LO_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_HI_ERR0_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_HI_ERR0_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_OFFSETDRIFT_COS_HI_ERR0_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_TRACK_LOCK_ERR0_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_TRACK_LOCK_ERR0_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_TRACK_LOCK_ERR0_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_TRACK_LOCK_ERR0_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_0_RESETVAL                         (0x00000000U)

/* DC_OFF_CFG1_0 */

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_SIN_AUTO_OFFSET_EN_MASK                (0x00000001U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_SIN_AUTO_OFFSET_EN_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_SIN_AUTO_OFFSET_EN_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_SIN_AUTO_OFFSET_EN_MAX                 (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_COS_AUTO_OFFSET_EN_MASK                (0x00000002U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_COS_AUTO_OFFSET_EN_SHIFT               (0x00000001U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_COS_AUTO_OFFSET_EN_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_COS_AUTO_OFFSET_EN_MAX                 (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MASK                    (0x00000008U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_SHIFT                   (0x00000003U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_CORR_ON_MAX                     (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_HYSTERESIS_MASK                 (0x000000F0U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_HYSTERESIS_SHIFT                (0x00000004U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_HYSTERESIS_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFFSET_HYSTERESIS_MAX                  (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MASK                 (0x00000100U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_SHIFT                (0x00000008U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_BANDPASSFILTER_ON_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_FILTORDER_MASK                         (0x0000F000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_FILTORDER_SHIFT                        (0x0000000CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_FILTORDER_RESETVAL                     (0x0000000AU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_FILTORDER_MAX                          (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_MASK                     (0x0F000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_SHIFT                    (0x00000018U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF1_MAX                      (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_MASK                     (0xF0000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_SHIFT                    (0x0000001CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_OFF_CAL_COEF2_MAX                      (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_0_RESETVAL                               (0x0000A000U)

/* DC_OFF_CFG2_0 */

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_MASK                   (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_SHIFT                  (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_SIN_MAN_OFF_ADJ_MAX                    (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_MASK                   (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_SHIFT                  (0x00000010U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_COS_MAN_OFF_ADJ_MAX                    (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_0_RESETVAL                               (0x00000000U)

/* DC_OFF0 */

#define CSL_RESOLVER_REGS_DC_OFF0_SIN_OFFSET_MASK                              (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DC_OFF0_SIN_OFFSET_SHIFT                             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF0_SIN_OFFSET_RESETVAL                          (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF0_SIN_OFFSET_MAX                               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_MASK                              (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_SHIFT                             (0x00000010U)
#define CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_RESETVAL                          (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF0_COS_OFFSET_MAX                               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF0_RESETVAL                                     (0x00000000U)

/* SAMPLE_CFG1_0 */

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_MASK                    (0x0000E000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_SHIFT                   (0x0000000DU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_PEAK_AVG_LIMIT_MAX                     (0x00000007U)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_MASK                 (0x00FF0000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_MASK             (0xFF000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_IDEAL_SAMPLE_TIME_OVR_MAX              (0x000000FFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_0_RESETVAL                               (0x00000000U)

/* SAMPLE_CFG2_0 */

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_MASK               (0x00000001U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_PEAK_AVG_LIMIT_DONE_MAX                (0x00000001U)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_MASK        (0x00001F00U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_SHIFT       (0x00000008U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_RESETVAL    (0x0000000AU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_BANDPASSFILTERSAMPLEADJUST_MAX         (0x0000001FU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_MASK              (0xFFFF0000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_SHIFT             (0x00000010U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_SAMPLE_DET_THRESHOLD_MAX               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_0_RESETVAL                               (0x00000A00U)

/* DEC_GF_CFG0 */

#define CSL_RESOLVER_REGS_DEC_GF_CFG0_NOISE_THRESHOLD_MASK                     (0x00000FFFU)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_NOISE_THRESHOLD_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_NOISE_THRESHOLD_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_NOISE_THRESHOLD_MAX                      (0x00000FFFU)

#define CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MASK                       (0x01000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_SHIFT                      (0x00000018U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_ENABLE_BOTTOM_MAX                        (0x00000001U)

#define CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_MASK                   (0x06000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_SHIFT                  (0x00000019U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG0_IDEAL_SAMPLE_MODE_MAX                    (0x00000003U)

#define CSL_RESOLVER_REGS_DEC_GF_CFG0_RESETVAL                                 (0x00000000U)

/* PG_EST_CFG1_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_MASK                    (0x0000000FU)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PG_TRAIN_LIMIT_MAX                     (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PHASEFILTCOEFF_MASK                    (0x000000F0U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PHASEFILTCOEFF_SHIFT                   (0x00000004U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PHASEFILTCOEFF_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_PHASEFILTCOEFF_MAX                     (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_MASK            (0x00010000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_SHIFT           (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_RESETVAL        (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AUTOPHASEGAINREADYDONE_MAX             (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF1_MASK                  (0x0F000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF1_SHIFT                 (0x00000018U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF1_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF1_MAX                   (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF2_MASK                  (0xF0000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF2_SHIFT                 (0x0000001CU)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF2_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_AGC_FILTER_COEF2_MAX                   (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_0_RESETVAL                               (0x00000000U)

/* PG_EST_CFG2_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MASK                  (0x00000002U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_SHIFT                 (0x00000001U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOGAINCONTROL0_MAX                   (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MASK                 (0x00000004U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_SHIFT                (0x00000002U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_AUTOPHASECONTROL0_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MASK              (0x00000008U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_SHIFT             (0x00000003U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_BYPASSPHASEGAINCORR0_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_MASK                      (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_SHIFT                     (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_RESETVAL                  (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_PHASECOSBYP0_MAX                       (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_0_RESETVAL                               (0x00000000U)

/* PG_EST_CFG3_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINSINBYP0_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_GAINCOSBYP0_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG3_0_RESETVAL                               (0x00000000U)

/* PG_EST_CFG4_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_RESETVAL            (0x00004000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PG_GLITCHTHRESHOLD_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_PHASEESTIMATEFINAL_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG4_0_RESETVAL                               (0x00004000U)

/* PG_EST_CFG5_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG5_0_COSSQACCFINAL0_MASK                    (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_0_COSSQACCFINAL0_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_0_COSSQACCFINAL0_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_0_COSSQACCFINAL0_MAX                     (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG5_0_RESETVAL                               (0x00000000U)

/* PG_EST_CFG6_0 */

#define CSL_RESOLVER_REGS_PG_EST_CFG6_0_SINSQACCFINAL0_MASK                    (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_0_SINSQACCFINAL0_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_0_SINSQACCFINAL0_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_0_SINSQACCFINAL0_MAX                     (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG6_0_RESETVAL                               (0x00000000U)

/* TRACK2_CFG1_0 */

#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_MASK                                (0x0000FF00U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_SHIFT                               (0x00000008U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_RESETVAL                            (0x0000003CU)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KI_MAX                                 (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KD_MASK                                (0x00FF0000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KD_SHIFT                               (0x00000010U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KD_RESETVAL                            (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KD_MAX                                 (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_MASK                          (0xFF000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_SHIFT                         (0x00000018U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_RESETVAL                      (0x00000005U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_KVELFILT_MAX                           (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_0_RESETVAL                               (0x05003C00U)

/* TRACK2_CFG2_0 */

#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_FEED_FWD_CORR_MASK                     (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_FEED_FWD_CORR_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_FEED_FWD_CORR_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_FEED_FWD_CORR_MAX                      (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_MASK                             (0x00000002U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_SHIFT                            (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_RESETVAL                         (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_BOOST_MAX                              (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_MASK                              (0x00000FF0U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_SHIFT                             (0x00000004U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_RESETVAL                          (0x00000006U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_KFFW_MAX                               (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_0_RESETVAL                               (0x00000062U)

/* TRACK2_CFG3_0 */

#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_MASK                       (0x000000FFU)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_VBOOSTCOEFF_MAX                        (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_MASK                             (0x0000FF00U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_SHIFT                            (0x00000008U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_KPDIV_MAX                              (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_BOOSTVEL_MASK                          (0x01000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_BOOSTVEL_SHIFT                         (0x00000018U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_BOOSTVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_BOOSTVEL_MAX                           (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_0_RESETVAL                               (0x00000000U)

/* ANGLE_ARCTAN_0 */

#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0_ANGLE_MASK                            (0x0000FFFFU)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0_ANGLE_SHIFT                           (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0_ANGLE_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0_ANGLE_MAX                             (0x0000FFFFU)

#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_0_RESETVAL                              (0x00000000U)

/* ANGLE_TRACK2_0 */

#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0_ANGLE_MASK                            (0x0000FFFFU)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0_ANGLE_SHIFT                           (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0_ANGLE_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0_ANGLE_MAX                             (0x0000FFFFU)

#define CSL_RESOLVER_REGS_ANGLE_TRACK2_0_RESETVAL                              (0x00000000U)

/* VELOCITY_TRACK2_0 */

#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0_VELOCITY_MASK                      (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0_VELOCITY_SHIFT                     (0x00000000U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0_VELOCITY_RESETVAL                  (0x00000000U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0_VELOCITY_MAX                       (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_0_RESETVAL                           (0x00000000U)

/* OFFSET_0 */


/* TRACK_ERR_0 */


/* TRACK_ERR2_0 */


/* DIAG0_0 */


/* DIAG1_0 */

#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_HI_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_0_OFFSETDRIFT_THRESHOLD_LO_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG1_0_RESETVAL                                     (0x00000000U)

/* DIAG2_0 */

#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_MASK                     (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_SIN_MAX                      (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_MASK                     (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_SHIFT                    (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_0_EXCFREQDETECTED_COS_MAX                      (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG2_0_RESETVAL                                     (0x00000000U)

/* DIAG3_0 */

#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_MASK               (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_HI_MAX                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_MASK               (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_0_EXCFREQDRIFT_THRESHOLD_LO_MAX                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG3_0_RESETVAL                                     (0x00000000U)

/* DIAG4_0 */

#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQ_LEVEL_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_MASK                (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_RESETVAL            (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG4_0_EXCFREQDRIFT_GLITCHCOUNT_MAX                 (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG4_0_RESETVAL                                     (0x00010000U)

/* DIAG5_0 */

#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_MASK                  (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_THRESHOLD_MAX                   (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_MASK                (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_RESETVAL            (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG5_0_LOWAMPLITUDE_GLITCHCOUNT_MAX                 (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG5_0_RESETVAL                                     (0x00010000U)

/* DIAG6_0 */

#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_MASK                        (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_SHIFT                       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_RESETVAL                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_SIN_MAX                         (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_MASK                        (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_SHIFT                       (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_RESETVAL                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_0_LOWAMPLITUDE_COS_MAX                         (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG6_0_RESETVAL                                     (0x00000000U)

/* DIAG7_0 */

#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_THRESHOLD_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_MASK               (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_RESETVAL           (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG7_0_HIGHAMPLITUDE_GLITCHCOUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG7_0_RESETVAL                                     (0x00010000U)

/* DIAG8_0 */

#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_SIN_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_0_HIGHAMPLITUDE_COS_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG8_0_RESETVAL                                     (0x00000000U)

/* DIAG9_0 */

#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_LO_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_0_SINSQCOSSQ_THRESHOLD_HI_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG9_0_RESETVAL                                     (0x00000000U)

/* DIAG10_0 */

#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_SINSQ_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_0_SINSQCOSSQ_COSSQ_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG10_0_RESETVAL                                    (0x00000000U)

/* DIAG11_0 */

#define CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_MASK                 (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_RESETVAL             (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG11_0_SINSQCOSSQ_GLITCHCOUNT_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG11_0_RESETVAL                                    (0x00000001U)

/* DIAG12_0 */

#define CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_MASK                          (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_SHIFT                         (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTFREQ_LEVEL_MAX                           (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_MASK                          (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_SHIFT                         (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_0_ROTPEAK_LEVEL_MAX                           (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG12_0_RESETVAL                                    (0x00000000U)

/* DIAG13_0 */

#define CSL_RESOLVER_REGS_DIAG13_0_SIN_POS_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_POS_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_POS_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_POS_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000002U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_NEG_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_0_COS_POS_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000004U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_POS_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000002U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_POS_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_POS_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_0_COS_NEG_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000008U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_NEG_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000003U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_NEG_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_NEG_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_MASK               (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_SIN_MULTI_ZC_ERROR_COUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_MASK               (0xFF000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_SHIFT              (0x00000018U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_0_COS_MULTI_ZC_ERROR_COUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG13_0_RESETVAL                                    (0x00000000U)

/* DIAG14_0 */

#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_HI_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_0_GAINDRIFT_THRESHOLD_LO_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG14_0_RESETVAL                                    (0x00000000U)

/* DIAG15_0 */

#define CSL_RESOLVER_REGS_DIAG15_0_GAINDRIFT_GLITCHCOUNT_MASK                  (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG15_0_GAINDRIFT_GLITCHCOUNT_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG15_0_GAINDRIFT_GLITCHCOUNT_RESETVAL              (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG15_0_GAINDRIFT_GLITCHCOUNT_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG15_0_RESETVAL                                    (0x00000001U)

/* DIAG16_0 */

#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_HI_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_0_PHASEDRIFT_THRESHOLD_LO_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG16_0_RESETVAL                                    (0x00000000U)

/* DIAG17_0 */

#define CSL_RESOLVER_REGS_DIAG17_0_PHASEDRIFT_GLITCHCOUNT_MASK                 (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG17_0_PHASEDRIFT_GLITCHCOUNT_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG17_0_PHASEDRIFT_GLITCHCOUNT_RESETVAL             (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG17_0_PHASEDRIFT_GLITCHCOUNT_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG17_0_RESETVAL                                    (0x00000001U)

/* DIAG18_0 */

#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_THRESHOLD_MASK                    (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_THRESHOLD_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_THRESHOLD_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_THRESHOLD_MAX                     (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_GLITCH_TIME_MASK                  (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_GLITCH_TIME_SHIFT                 (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_GLITCH_TIME_RESETVAL              (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG18_0_TRACK_ERR_GLITCH_TIME_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG18_0_RESETVAL                                    (0x00010000U)

/* DIAG19_0 */


/* DIAG20_0 */


/* DIAG21_0 */


/* OBS_ADC_0 */

#define CSL_RESOLVER_REGS_OBS_ADC_0_SIN_ADC_MASK                               (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_0_SIN_ADC_SHIFT                              (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_0_SIN_ADC_RESETVAL                           (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_0_SIN_ADC_MAX                                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_0_COS_ADC_MASK                               (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_0_COS_ADC_SHIFT                              (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_0_COS_ADC_RESETVAL                           (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_0_COS_ADC_MAX                                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_0_RESETVAL                                   (0x00000000U)

/* OBS_ADC_REC_0 */

#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_SIN_REC_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_SIN_REC_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_SIN_REC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_SIN_REC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_COS_REC_MASK                           (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_COS_REC_SHIFT                          (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_COS_REC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_COS_REC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_REC_0_RESETVAL                               (0x00000000U)

/* OBS_ADC_DC_0 */

#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_SIN_DC_MASK                             (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_SIN_DC_SHIFT                            (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_SIN_DC_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_SIN_DC_MAX                              (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_COS_DC_MASK                             (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_COS_DC_SHIFT                            (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_COS_DC_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_COS_DC_MAX                              (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_DC_0_RESETVAL                                (0x00000000U)

/* OBS_ADC_PGC_0 */

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_SIN_PGC_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_SIN_PGC_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_SIN_PGC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_SIN_PGC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_COS_PGC_MASK                           (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_COS_PGC_SHIFT                          (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_COS_PGC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_COS_PGC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_0_RESETVAL                               (0x00000000U)

/* OBS_NOISE_VAL_0 */

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_SIN_NOISE_VALUE_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_SIN_NOISE_VALUE_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_SIN_NOISE_VALUE_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_SIN_NOISE_VALUE_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_COS_NOISE_VALUE_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_COS_NOISE_VALUE_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_COS_NOISE_VALUE_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_COS_NOISE_VALUE_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_0_RESETVAL                             (0x00000000U)

/* OBS_PEAKHISTOGRAM3_0_0 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_MASK         (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_SHIFT        (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM0_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_MASK         (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_SHIFT        (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM1_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_MASK         (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_SHIFT        (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM2_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_MASK         (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_SHIFT        (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_PEAKHISTOGRAM3_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_0_RESETVAL                      (0x00000000U)

/* OBS_PEAKHISTOGRAM7_4_0 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_MASK         (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_SHIFT        (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM4_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_MASK         (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_SHIFT        (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM5_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_MASK         (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_SHIFT        (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM6_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_MASK         (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_SHIFT        (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_PEAKHISTOGRAM7_0_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_0_RESETVAL                      (0x00000000U)

/* OBS_PEAKHISTOGRAM11_8_0 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_MASK        (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_SHIFT       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM8_0_MAX         (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_MASK        (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_SHIFT       (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM9_0_MAX         (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_MASK       (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM10_0_MAX        (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_MASK       (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_SHIFT      (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_PEAKHISTOGRAM11_0_MAX        (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_0_RESETVAL                     (0x00000000U)

/* OBS_PEAKHISTOGRAM15_12_0 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_MASK      (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_SHIFT     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM12_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_MASK      (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_SHIFT     (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM13_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_MASK      (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_SHIFT     (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM14_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_MASK      (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_SHIFT     (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_PEAKHISTOGRAM15_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_0_RESETVAL                    (0x00000000U)

/* OBS_PEAKHISTOGRAM19_16_0 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_MASK      (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_SHIFT     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM16_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_MASK      (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_SHIFT     (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM17_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_MASK      (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_SHIFT     (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM18_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_MASK      (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_SHIFT     (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_PEAKHISTOGRAM19_0_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_0_RESETVAL                    (0x00000000U)

/* IRQSTATUS_RAW_SYS_1 */

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_LOWAMPLITUDE_ERR1_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_LOWAMPLITUDE_ERR1_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_LOWAMPLITUDE_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_LOWAMPLITUDE_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_LO_ERR1_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_LO_ERR1_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_LO_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_LO_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_HI_ERR1_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_HI_ERR1_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_HI_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SINSQCOSSQ_HI_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_HI_ERR1_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_HI_ERR1_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_HI_ERR1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_EXCFREQDRIFT_HI_ERR1_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_LO_ERR1_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_LO_ERR1_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_LO_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_LO_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_HI_ERR1_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_HI_ERR1_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_HI_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_PHASEDRIFT_COS_HI_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_LO_ERR1_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_LO_ERR1_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_HI_ERR1_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_HI_ERR1_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_SIN_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_LO_ERR1_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_LO_ERR1_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_HI_ERR1_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_HI_ERR1_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_GAINDRIFT_COS_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_LO_ERR1_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_HI_ERR1_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_TRACK_LOCK_ERR1_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_TRACK_LOCK_ERR1_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_TRACK_LOCK_ERR1_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_TRACK_LOCK_ERR1_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_RAW_SYS_1_RESETVAL                         (0x00000000U)

/* IRQSTATUS_SYS_1 */

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_LOWAMPLITUDE_ERR1_MASK               (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_LOWAMPLITUDE_ERR1_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_LOWAMPLITUDE_ERR1_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_LOWAMPLITUDE_ERR1_MAX                (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MASK    (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_SHIFT   (0x00000001U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MASK    (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_SHIFT   (0x00000002U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_LO_ERR1_MASK              (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_LO_ERR1_SHIFT             (0x00000003U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_LO_ERR1_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_LO_ERR1_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_HI_ERR1_MASK              (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_HI_ERR1_SHIFT             (0x00000004U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_HI_ERR1_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SINSQCOSSQ_HI_ERR1_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MASK         (0x00000020U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_MULTI_ZC_ERROR_ERR1_SHIFT        (0x00000005U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_MULTI_ZC_ERROR_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MASK         (0x00000040U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_SHIFT        (0x00000006U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MASK   (0x00000080U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT  (0x00000007U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MASK   (0x00000100U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT  (0x00000008U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MASK   (0x00000200U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT  (0x00000009U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MASK   (0x00000400U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT  (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MAX    (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MASK        (0x00000800U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_SHIFT       (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MASK        (0x00001000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_SHIFT       (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_HI_ERR1_MASK            (0x00002000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_HI_ERR1_SHIFT           (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_HI_ERR1_RESETVAL        (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_EXCFREQDRIFT_HI_ERR1_MAX             (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_LO_ERR1_MASK          (0x00004000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_LO_ERR1_SHIFT         (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_LO_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_LO_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_HI_ERR1_MASK          (0x00008000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_HI_ERR1_SHIFT         (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_HI_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_PHASEDRIFT_COS_HI_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_LO_ERR1_MASK           (0x00010000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_LO_ERR1_SHIFT          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_LO_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_LO_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_HI_ERR1_MASK           (0x00020000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_HI_ERR1_SHIFT          (0x00000011U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_HI_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_SIN_HI_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_LO_ERR1_MASK           (0x00040000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_LO_ERR1_SHIFT          (0x00000012U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_LO_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_LO_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_HI_ERR1_MASK           (0x00080000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_HI_ERR1_SHIFT          (0x00000013U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_HI_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_GAINDRIFT_COS_HI_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MASK         (0x00100000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_SHIFT        (0x00000014U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MASK         (0x00200000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_SHIFT        (0x00000015U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MASK         (0x00400000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_LO_ERR1_SHIFT        (0x00000016U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_LO_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MASK         (0x00800000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_HI_ERR1_SHIFT        (0x00000017U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_HI_ERR1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MAX          (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_TRACK_LOCK_ERR1_MASK                 (0x01000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_TRACK_LOCK_ERR1_SHIFT                (0x00000018U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_TRACK_LOCK_ERR1_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_TRACK_LOCK_ERR1_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_IRQSTATUS_SYS_1_RESETVAL                             (0x00000000U)

/* IRQENABLE_SET_SYS_1 */

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_LOWAMPLITUDE_ERR1_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_LOWAMPLITUDE_ERR1_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_LOWAMPLITUDE_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_LOWAMPLITUDE_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_LO_ERR1_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_LO_ERR1_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_LO_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_LO_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_HI_ERR1_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_HI_ERR1_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_HI_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SINSQCOSSQ_HI_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_HI_ERR1_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_HI_ERR1_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_HI_ERR1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_EXCFREQDRIFT_HI_ERR1_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_LO_ERR1_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_LO_ERR1_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_LO_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_LO_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_HI_ERR1_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_HI_ERR1_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_HI_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_PHASEDRIFT_COS_HI_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_LO_ERR1_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_LO_ERR1_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_HI_ERR1_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_HI_ERR1_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_SIN_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_LO_ERR1_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_LO_ERR1_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_HI_ERR1_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_HI_ERR1_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_GAINDRIFT_COS_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_LO_ERR1_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_HI_ERR1_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_TRACK_LOCK_ERR1_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_TRACK_LOCK_ERR1_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_TRACK_LOCK_ERR1_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_TRACK_LOCK_ERR1_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_SET_SYS_1_RESETVAL                         (0x00000000U)

/* IRQENABLE_CLR_SYS_1 */

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_LOWAMPLITUDE_ERR1_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_LOWAMPLITUDE_ERR1_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_LOWAMPLITUDE_ERR1_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_LOWAMPLITUDE_ERR1_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MASK (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_SHIFT (0x00000001U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_COS_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MASK (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_SHIFT (0x00000002U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_HIGHAMPLITUDE_SIN_FAULT_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_LO_ERR1_MASK          (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_LO_ERR1_SHIFT         (0x00000003U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_LO_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_LO_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_HI_ERR1_MASK          (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_HI_ERR1_SHIFT         (0x00000004U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_HI_ERR1_RESETVAL      (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SINSQCOSSQ_HI_ERR1_MAX           (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MASK     (0x00000020U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000005U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MASK     (0x00000040U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_SHIFT    (0x00000006U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_MULTI_ZC_ERROR_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000080U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000007U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000100U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000008U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_COS_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000200U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x00000009U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MASK (0x00000400U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_SHIFT (0x0000000AU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_SIN_POS_ZC_PEAK_MISMATCH_ERR1_MAX (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MASK    (0x00000800U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_SHIFT   (0x0000000BU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_SIN_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MASK    (0x00001000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_SHIFT   (0x0000000CU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_COS_LO_ERR1_MAX     (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_HI_ERR1_MASK        (0x00002000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_HI_ERR1_SHIFT       (0x0000000DU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_HI_ERR1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_EXCFREQDRIFT_HI_ERR1_MAX         (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_LO_ERR1_MASK      (0x00004000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_LO_ERR1_SHIFT     (0x0000000EU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_LO_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_LO_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_HI_ERR1_MASK      (0x00008000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_HI_ERR1_SHIFT     (0x0000000FU)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_HI_ERR1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_PHASEDRIFT_COS_HI_ERR1_MAX       (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_LO_ERR1_MASK       (0x00010000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_LO_ERR1_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_HI_ERR1_MASK       (0x00020000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_HI_ERR1_SHIFT      (0x00000011U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_SIN_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_LO_ERR1_MASK       (0x00040000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_LO_ERR1_SHIFT      (0x00000012U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_LO_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_LO_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_HI_ERR1_MASK       (0x00080000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_HI_ERR1_SHIFT      (0x00000013U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_HI_ERR1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_GAINDRIFT_COS_HI_ERR1_MAX        (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MASK     (0x00100000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_SHIFT    (0x00000014U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MASK     (0x00200000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_SHIFT    (0x00000015U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_SIN_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MASK     (0x00400000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_LO_ERR1_SHIFT    (0x00000016U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_LO_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_LO_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MASK     (0x00800000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_HI_ERR1_SHIFT    (0x00000017U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_HI_ERR1_RESETVAL (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_OFFSETDRIFT_COS_HI_ERR1_MAX      (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_TRACK_LOCK_ERR1_MASK             (0x01000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_TRACK_LOCK_ERR1_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_TRACK_LOCK_ERR1_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_TRACK_LOCK_ERR1_MAX              (0x00000001U)

#define CSL_RESOLVER_REGS_IRQENABLE_CLR_SYS_1_RESETVAL                         (0x00000000U)

/* DC_OFF_CFG1_1 */

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_SIN_AUTO_OFFSET_EN_MASK                (0x00000001U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_SIN_AUTO_OFFSET_EN_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_SIN_AUTO_OFFSET_EN_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_SIN_AUTO_OFFSET_EN_MAX                 (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_COS_AUTO_OFFSET_EN_MASK                (0x00000002U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_COS_AUTO_OFFSET_EN_SHIFT               (0x00000001U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_COS_AUTO_OFFSET_EN_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_COS_AUTO_OFFSET_EN_MAX                 (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_CORR_ON_MASK                    (0x00000008U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_CORR_ON_SHIFT                   (0x00000003U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_CORR_ON_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_CORR_ON_MAX                     (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_MASK                 (0x000000F0U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_SHIFT                (0x00000004U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFFSET_HYSTERESIS_MAX                  (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_BANDPASSFILTER_ON_MASK                 (0x00000100U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_BANDPASSFILTER_ON_SHIFT                (0x00000008U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_BANDPASSFILTER_ON_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_BANDPASSFILTER_ON_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_FILTORDER_MASK                         (0x0000F000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_FILTORDER_SHIFT                        (0x0000000CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_FILTORDER_RESETVAL                     (0x0000000AU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_FILTORDER_MAX                          (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF1_MASK                     (0x0F000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF1_SHIFT                    (0x00000018U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF1_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF1_MAX                      (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF2_MASK                     (0xF0000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF2_SHIFT                    (0x0000001CU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF2_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_OFF_CAL_COEF2_MAX                      (0x0000000FU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG1_1_RESETVAL                               (0x0000A000U)

/* DC_OFF_CFG2_1 */

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_SIN_MAN_OFF_ADJ_MASK                   (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_SIN_MAN_OFF_ADJ_SHIFT                  (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_SIN_MAN_OFF_ADJ_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_SIN_MAN_OFF_ADJ_MAX                    (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_COS_MAN_OFF_ADJ_MASK                   (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_COS_MAN_OFF_ADJ_SHIFT                  (0x00000010U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_COS_MAN_OFF_ADJ_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_COS_MAN_OFF_ADJ_MAX                    (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF_CFG2_1_RESETVAL                               (0x00000000U)

/* DC_OFF1 */

#define CSL_RESOLVER_REGS_DC_OFF1_SIN_OFFSET_MASK                              (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DC_OFF1_SIN_OFFSET_SHIFT                             (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF1_SIN_OFFSET_RESETVAL                          (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF1_SIN_OFFSET_MAX                               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF1_COS_OFFSET_MASK                              (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DC_OFF1_COS_OFFSET_SHIFT                             (0x00000010U)
#define CSL_RESOLVER_REGS_DC_OFF1_COS_OFFSET_RESETVAL                          (0x00000000U)
#define CSL_RESOLVER_REGS_DC_OFF1_COS_OFFSET_MAX                               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DC_OFF1_RESETVAL                                     (0x00000000U)

/* SAMPLE_CFG1_1 */

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_PEAK_AVG_LIMIT_MASK                    (0x0000E000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_PEAK_AVG_LIMIT_SHIFT                   (0x0000000DU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_PEAK_AVG_LIMIT_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_PEAK_AVG_LIMIT_MAX                     (0x00000007U)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_MASK                 (0x00FF0000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_OVR_MASK             (0xFF000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_OVR_SHIFT            (0x00000018U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_OVR_RESETVAL         (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_IDEAL_SAMPLE_TIME_OVR_MAX              (0x000000FFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG1_1_RESETVAL                               (0x00000000U)

/* SAMPLE_CFG2_1 */

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_PEAK_AVG_LIMIT_DONE_MASK               (0x00000001U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_PEAK_AVG_LIMIT_DONE_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_PEAK_AVG_LIMIT_DONE_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_PEAK_AVG_LIMIT_DONE_MAX                (0x00000001U)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_BANDPASSFILTERSAMPLEADJUST_MASK        (0x00001F00U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_BANDPASSFILTERSAMPLEADJUST_SHIFT       (0x00000008U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_BANDPASSFILTERSAMPLEADJUST_RESETVAL    (0x0000000AU)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_BANDPASSFILTERSAMPLEADJUST_MAX         (0x0000001FU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_SAMPLE_DET_THRESHOLD_MASK              (0xFFFF0000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_SAMPLE_DET_THRESHOLD_SHIFT             (0x00000010U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_SAMPLE_DET_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_SAMPLE_DET_THRESHOLD_MAX               (0x0000FFFFU)

#define CSL_RESOLVER_REGS_SAMPLE_CFG2_1_RESETVAL                               (0x00000A00U)

/* DEC_GF_CFG1 */

#define CSL_RESOLVER_REGS_DEC_GF_CFG1_NOISE_THRESHOLD_MASK                     (0x00000FFFU)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_NOISE_THRESHOLD_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_NOISE_THRESHOLD_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_NOISE_THRESHOLD_MAX                      (0x00000FFFU)

#define CSL_RESOLVER_REGS_DEC_GF_CFG1_ENABLE_BOTTOM_MASK                       (0x01000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_ENABLE_BOTTOM_SHIFT                      (0x00000018U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_ENABLE_BOTTOM_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_ENABLE_BOTTOM_MAX                        (0x00000001U)

#define CSL_RESOLVER_REGS_DEC_GF_CFG1_IDEAL_SAMPLE_MODE_MASK                   (0x06000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_IDEAL_SAMPLE_MODE_SHIFT                  (0x00000019U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_IDEAL_SAMPLE_MODE_RESETVAL               (0x00000000U)
#define CSL_RESOLVER_REGS_DEC_GF_CFG1_IDEAL_SAMPLE_MODE_MAX                    (0x00000003U)

#define CSL_RESOLVER_REGS_DEC_GF_CFG1_RESETVAL                                 (0x00000000U)

/* PG_EST_CFG1_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PG_TRAIN_LIMIT_MASK                    (0x0000000FU)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PG_TRAIN_LIMIT_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PG_TRAIN_LIMIT_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PG_TRAIN_LIMIT_MAX                     (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PHASEFILTCOEFF_MASK                    (0x000000F0U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PHASEFILTCOEFF_SHIFT                   (0x00000004U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PHASEFILTCOEFF_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_PHASEFILTCOEFF_MAX                     (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AUTOPHASEGAINREADYDONE_MASK            (0x00010000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AUTOPHASEGAINREADYDONE_SHIFT           (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AUTOPHASEGAINREADYDONE_RESETVAL        (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AUTOPHASEGAINREADYDONE_MAX             (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF1_MASK                  (0x0F000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF1_SHIFT                 (0x00000018U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF1_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF1_MAX                   (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF2_MASK                  (0xF0000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF2_SHIFT                 (0x0000001CU)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF2_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_AGC_FILTER_COEF2_MAX                   (0x0000000FU)

#define CSL_RESOLVER_REGS_PG_EST_CFG1_1_RESETVAL                               (0x00000000U)

/* PG_EST_CFG2_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOGAINCONTROL1_MASK                  (0x00000002U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOGAINCONTROL1_SHIFT                 (0x00000001U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOGAINCONTROL1_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOGAINCONTROL1_MAX                   (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOPHASECONTROL1_MASK                 (0x00000004U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOPHASECONTROL1_SHIFT                (0x00000002U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOPHASECONTROL1_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_AUTOPHASECONTROL1_MAX                  (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_BYPASSPHASEGAINCORR1_MASK              (0x00000008U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_BYPASSPHASEGAINCORR1_SHIFT             (0x00000003U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_BYPASSPHASEGAINCORR1_RESETVAL          (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_BYPASSPHASEGAINCORR1_MAX               (0x00000001U)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_PHASECOSBYP1_MASK                      (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_PHASECOSBYP1_SHIFT                     (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_PHASECOSBYP1_RESETVAL                  (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_PHASECOSBYP1_MAX                       (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG2_1_RESETVAL                               (0x00000000U)

/* PG_EST_CFG3_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINSINBYP1_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINSINBYP1_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINSINBYP1_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINSINBYP1_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINCOSBYP1_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINCOSBYP1_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINCOSBYP1_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_GAINCOSBYP1_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG3_1_RESETVAL                               (0x00000000U)

/* PG_EST_CFG4_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PG_GLITCHTHRESHOLD_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PG_GLITCHTHRESHOLD_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PG_GLITCHTHRESHOLD_RESETVAL            (0x00004000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PG_GLITCHTHRESHOLD_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PHASEESTIMATEFINAL_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PHASEESTIMATEFINAL_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PHASEESTIMATEFINAL_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_PHASEESTIMATEFINAL_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG4_1_RESETVAL                               (0x00004000U)

/* PG_EST_CFG5_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG5_1_COSSQACCFINAL1_MASK                    (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_1_COSSQACCFINAL1_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_1_COSSQACCFINAL1_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG5_1_COSSQACCFINAL1_MAX                     (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG5_1_RESETVAL                               (0x00000000U)

/* PG_EST_CFG6_1 */

#define CSL_RESOLVER_REGS_PG_EST_CFG6_1_SINSQACCFINAL1_MASK                    (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_1_SINSQACCFINAL1_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_1_SINSQACCFINAL1_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_PG_EST_CFG6_1_SINSQACCFINAL1_MAX                     (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_PG_EST_CFG6_1_RESETVAL                               (0x00000000U)

/* TRACK2_CFG1_1 */

#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KI_MASK                                (0x0000FF00U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KI_SHIFT                               (0x00000008U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KI_RESETVAL                            (0x0000003CU)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KI_MAX                                 (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KD_MASK                                (0x00FF0000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KD_SHIFT                               (0x00000010U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KD_RESETVAL                            (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KD_MAX                                 (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KVELFILT_MASK                          (0xFF000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KVELFILT_SHIFT                         (0x00000018U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KVELFILT_RESETVAL                      (0x00000005U)
#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_KVELFILT_MAX                           (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG1_1_RESETVAL                               (0x05003C00U)

/* TRACK2_CFG2_1 */

#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_FEED_FWD_CORR_MASK                     (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_FEED_FWD_CORR_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_FEED_FWD_CORR_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_FEED_FWD_CORR_MAX                      (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_BOOST_MASK                             (0x00000002U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_BOOST_SHIFT                            (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_BOOST_RESETVAL                         (0x00000001U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_BOOST_MAX                              (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_KFFW_MASK                              (0x00000FF0U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_KFFW_SHIFT                             (0x00000004U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_KFFW_RESETVAL                          (0x00000006U)
#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_KFFW_MAX                               (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG2_1_RESETVAL                               (0x00000062U)

/* TRACK2_CFG3_1 */

#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_VBOOSTCOEFF_MASK                       (0x000000FFU)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_VBOOSTCOEFF_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_VBOOSTCOEFF_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_VBOOSTCOEFF_MAX                        (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_KPDIV_MASK                             (0x0000FF00U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_KPDIV_SHIFT                            (0x00000008U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_KPDIV_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_KPDIV_MAX                              (0x000000FFU)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_BOOSTVEL_MASK                          (0x01000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_BOOSTVEL_SHIFT                         (0x00000018U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_BOOSTVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_BOOSTVEL_MAX                           (0x00000001U)

#define CSL_RESOLVER_REGS_TRACK2_CFG3_1_RESETVAL                               (0x00000000U)

/* ANGLE_ARCTAN_1 */

#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1_ANGLE_MASK                            (0x0000FFFFU)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1_ANGLE_SHIFT                           (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1_ANGLE_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1_ANGLE_MAX                             (0x0000FFFFU)

#define CSL_RESOLVER_REGS_ANGLE_ARCTAN_1_RESETVAL                              (0x00000000U)

/* ANGLE_TRACK2_1 */

#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1_ANGLE_MASK                            (0x0000FFFFU)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1_ANGLE_SHIFT                           (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1_ANGLE_RESETVAL                        (0x00000000U)
#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1_ANGLE_MAX                             (0x0000FFFFU)

#define CSL_RESOLVER_REGS_ANGLE_TRACK2_1_RESETVAL                              (0x00000000U)

/* VELOCITY_TRACK2_1 */

#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1_VELOCITY_MASK                      (0xFFFFFFFFU)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1_VELOCITY_SHIFT                     (0x00000000U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1_VELOCITY_RESETVAL                  (0x00000000U)
#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1_VELOCITY_MAX                       (0xFFFFFFFFU)

#define CSL_RESOLVER_REGS_VELOCITY_TRACK2_1_RESETVAL                           (0x00000000U)

/* OFFSET_1 */


/* TRACK_ERR_1 */


/* TRACK_ERR2_1 */


/* DIAG0_1 */


/* DIAG1_1 */

#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_HI_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_HI_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_HI_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_HI_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_LO_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_LO_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_LO_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG1_1_OFFSETDRIFT_THRESHOLD_LO_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG1_1_RESETVAL                                     (0x00000000U)

/* DIAG2_1 */

#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_SIN_MASK                     (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_SIN_SHIFT                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_SIN_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_SIN_MAX                      (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_COS_MASK                     (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_COS_SHIFT                    (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_COS_RESETVAL                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG2_1_EXCFREQDETECTED_COS_MAX                      (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG2_1_RESETVAL                                     (0x00000000U)

/* DIAG3_1 */

#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_HI_MASK               (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_HI_SHIFT              (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_HI_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_HI_MAX                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_LO_MASK               (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_LO_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_LO_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG3_1_EXCFREQDRIFT_THRESHOLD_LO_MAX                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG3_1_RESETVAL                                     (0x00000000U)

/* DIAG4_1 */

#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQ_LEVEL_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQ_LEVEL_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQ_LEVEL_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQ_LEVEL_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQDRIFT_GLITCHCOUNT_MASK                (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQDRIFT_GLITCHCOUNT_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQDRIFT_GLITCHCOUNT_RESETVAL            (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG4_1_EXCFREQDRIFT_GLITCHCOUNT_MAX                 (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG4_1_RESETVAL                                     (0x00010000U)

/* DIAG5_1 */

#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_THRESHOLD_MASK                  (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_THRESHOLD_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_THRESHOLD_RESETVAL              (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_THRESHOLD_MAX                   (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_GLITCHCOUNT_MASK                (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_GLITCHCOUNT_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_GLITCHCOUNT_RESETVAL            (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG5_1_LOWAMPLITUDE_GLITCHCOUNT_MAX                 (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG5_1_RESETVAL                                     (0x00010000U)

/* DIAG6_1 */

#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_SIN_MASK                        (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_SIN_SHIFT                       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_SIN_RESETVAL                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_SIN_MAX                         (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_COS_MASK                        (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_COS_SHIFT                       (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_COS_RESETVAL                    (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG6_1_LOWAMPLITUDE_COS_MAX                         (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG6_1_RESETVAL                                     (0x00000000U)

/* DIAG7_1 */

#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_THRESHOLD_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_THRESHOLD_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_THRESHOLD_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_THRESHOLD_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_GLITCHCOUNT_MASK               (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_GLITCHCOUNT_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_GLITCHCOUNT_RESETVAL           (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG7_1_HIGHAMPLITUDE_GLITCHCOUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG7_1_RESETVAL                                     (0x00010000U)

/* DIAG8_1 */

#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_SIN_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_SIN_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_SIN_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_SIN_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_COS_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_COS_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_COS_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG8_1_HIGHAMPLITUDE_COS_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG8_1_RESETVAL                                     (0x00000000U)

/* DIAG9_1 */

#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_LO_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_LO_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_LO_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_LO_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_HI_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_HI_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_HI_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG9_1_SINSQCOSSQ_THRESHOLD_HI_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG9_1_RESETVAL                                     (0x00000000U)

/* DIAG10_1 */

#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_SINSQ_MASK                       (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_SINSQ_SHIFT                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_SINSQ_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_SINSQ_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_COSSQ_MASK                       (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_COSSQ_SHIFT                      (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_COSSQ_RESETVAL                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG10_1_SINSQCOSSQ_COSSQ_MAX                        (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG10_1_RESETVAL                                    (0x00000000U)

/* DIAG11_1 */

#define CSL_RESOLVER_REGS_DIAG11_1_SINSQCOSSQ_GLITCHCOUNT_MASK                 (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG11_1_SINSQCOSSQ_GLITCHCOUNT_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG11_1_SINSQCOSSQ_GLITCHCOUNT_RESETVAL             (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG11_1_SINSQCOSSQ_GLITCHCOUNT_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG11_1_RESETVAL                                    (0x00000001U)

/* DIAG12_1 */

#define CSL_RESOLVER_REGS_DIAG12_1_ROTFREQ_LEVEL_MASK                          (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTFREQ_LEVEL_SHIFT                         (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTFREQ_LEVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTFREQ_LEVEL_MAX                           (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG12_1_ROTPEAK_LEVEL_MASK                          (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTPEAK_LEVEL_SHIFT                         (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTPEAK_LEVEL_RESETVAL                      (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG12_1_ROTPEAK_LEVEL_MAX                           (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG12_1_RESETVAL                                    (0x00000000U)

/* DIAG13_1 */

#define CSL_RESOLVER_REGS_DIAG13_1_SIN_POS_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_POS_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_POS_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_POS_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000002U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_NEG_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_1_COS_POS_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000004U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_POS_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000002U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_POS_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_POS_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_1_COS_NEG_ZC_PEAK_MISMATCH_ERR_MASK           (0x00000008U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_NEG_ZC_PEAK_MISMATCH_ERR_SHIFT          (0x00000003U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_NEG_ZC_PEAK_MISMATCH_ERR_RESETVAL       (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_NEG_ZC_PEAK_MISMATCH_ERR_MAX            (0x00000001U)

#define CSL_RESOLVER_REGS_DIAG13_1_SIN_MULTI_ZC_ERROR_COUNT_MASK               (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_MULTI_ZC_ERROR_COUNT_SHIFT              (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_MULTI_ZC_ERROR_COUNT_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_SIN_MULTI_ZC_ERROR_COUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG13_1_COS_MULTI_ZC_ERROR_COUNT_MASK               (0xFF000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_MULTI_ZC_ERROR_COUNT_SHIFT              (0x00000018U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_MULTI_ZC_ERROR_COUNT_RESETVAL           (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG13_1_COS_MULTI_ZC_ERROR_COUNT_MAX                (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG13_1_RESETVAL                                    (0x00000000U)

/* DIAG14_1 */

#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_HI_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_HI_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_HI_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_HI_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_LO_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_LO_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_LO_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG14_1_GAINDRIFT_THRESHOLD_LO_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG14_1_RESETVAL                                    (0x00000000U)

/* DIAG15_1 */

#define CSL_RESOLVER_REGS_DIAG15_1_GAINDRIFT_GLITCHCOUNT_MASK                  (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG15_1_GAINDRIFT_GLITCHCOUNT_SHIFT                 (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG15_1_GAINDRIFT_GLITCHCOUNT_RESETVAL              (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG15_1_GAINDRIFT_GLITCHCOUNT_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG15_1_RESETVAL                                    (0x00000001U)

/* DIAG16_1 */

#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_HI_MASK                (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_HI_SHIFT               (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_HI_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_HI_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_LO_MASK                (0xFFFF0000U)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_LO_SHIFT               (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_LO_RESETVAL            (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG16_1_PHASEDRIFT_THRESHOLD_LO_MAX                 (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG16_1_RESETVAL                                    (0x00000000U)

/* DIAG17_1 */

#define CSL_RESOLVER_REGS_DIAG17_1_PHASEDRIFT_GLITCHCOUNT_MASK                 (0x000000FFU)
#define CSL_RESOLVER_REGS_DIAG17_1_PHASEDRIFT_GLITCHCOUNT_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG17_1_PHASEDRIFT_GLITCHCOUNT_RESETVAL             (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG17_1_PHASEDRIFT_GLITCHCOUNT_MAX                  (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG17_1_RESETVAL                                    (0x00000001U)

/* DIAG18_1 */

#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_THRESHOLD_MASK                    (0x0000FFFFU)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_THRESHOLD_SHIFT                   (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_THRESHOLD_RESETVAL                (0x00000000U)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_THRESHOLD_MAX                     (0x0000FFFFU)

#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_GLITCH_TIME_MASK                  (0x00FF0000U)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_GLITCH_TIME_SHIFT                 (0x00000010U)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_GLITCH_TIME_RESETVAL              (0x00000001U)
#define CSL_RESOLVER_REGS_DIAG18_1_TRACK_ERR_GLITCH_TIME_MAX                   (0x000000FFU)

#define CSL_RESOLVER_REGS_DIAG18_1_RESETVAL                                    (0x00010000U)

/* DIAG19_1 */


/* DIAG20_1 */


/* DIAG21_1 */


/* OBS_ADC_1 */

#define CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_MASK                               (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_SHIFT                              (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_RESETVAL                           (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_1_SIN_ADC_MAX                                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_MASK                               (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_SHIFT                              (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_RESETVAL                           (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_1_COS_ADC_MAX                                (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_1_RESETVAL                                   (0x00000000U)

/* OBS_ADC_REC_1 */

#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_SIN_REC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_MASK                           (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_SHIFT                          (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_COS_REC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_REC_1_RESETVAL                               (0x00000000U)

/* OBS_ADC_DC_1 */

#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_MASK                             (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_SHIFT                            (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_SIN_DC_MAX                              (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_MASK                             (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_SHIFT                            (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_RESETVAL                         (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_COS_DC_MAX                              (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_DC_1_RESETVAL                                (0x00000000U)

/* OBS_ADC_PGC_1 */

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_MASK                           (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_SHIFT                          (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_SIN_PGC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_MASK                           (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_SHIFT                          (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_RESETVAL                       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_COS_PGC_MAX                            (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_ADC_PGC_1_RESETVAL                               (0x00000000U)

/* OBS_NOISE_VAL_1 */

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_SIN_NOISE_VALUE_MASK                 (0x0000FFFFU)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_SIN_NOISE_VALUE_SHIFT                (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_SIN_NOISE_VALUE_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_SIN_NOISE_VALUE_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_COS_NOISE_VALUE_MASK                 (0xFFFF0000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_COS_NOISE_VALUE_SHIFT                (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_COS_NOISE_VALUE_RESETVAL             (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_COS_NOISE_VALUE_MAX                  (0x0000FFFFU)

#define CSL_RESOLVER_REGS_OBS_NOISE_VAL_1_RESETVAL                             (0x00000000U)

/* OBS_PEAKHISTOGRAM3_0_1 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM0_1_MASK         (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM0_1_SHIFT        (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM0_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM0_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM1_1_MASK         (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM1_1_SHIFT        (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM1_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM1_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM2_1_MASK         (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM2_1_SHIFT        (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM2_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM2_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM3_1_MASK         (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM3_1_SHIFT        (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM3_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_PEAKHISTOGRAM3_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM3_0_1_RESETVAL                      (0x00000000U)

/* OBS_PEAKHISTOGRAM7_4_1 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM4_1_MASK         (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM4_1_SHIFT        (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM4_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM4_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM5_1_MASK         (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM5_1_SHIFT        (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM5_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM5_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM6_1_MASK         (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM6_1_SHIFT        (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM6_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM6_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM7_1_MASK         (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM7_1_SHIFT        (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM7_1_RESETVAL     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_PEAKHISTOGRAM7_1_MAX          (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM7_4_1_RESETVAL                      (0x00000000U)

/* OBS_PEAKHISTOGRAM11_8_1 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM8_1_MASK        (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM8_1_SHIFT       (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM8_1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM8_1_MAX         (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM9_1_MASK        (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM9_1_SHIFT       (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM9_1_RESETVAL    (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM9_1_MAX         (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM10_1_MASK       (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM10_1_SHIFT      (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM10_1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM10_1_MAX        (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM11_1_MASK       (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM11_1_SHIFT      (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM11_1_RESETVAL   (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_PEAKHISTOGRAM11_1_MAX        (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM11_8_1_RESETVAL                     (0x00000000U)

/* OBS_PEAKHISTOGRAM15_12_1 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM12_1_MASK      (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM12_1_SHIFT     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM12_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM12_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM13_1_MASK      (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM13_1_SHIFT     (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM13_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM13_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM14_1_MASK      (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM14_1_SHIFT     (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM14_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM14_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM15_1_MASK      (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM15_1_SHIFT     (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM15_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_PEAKHISTOGRAM15_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM15_12_1_RESETVAL                    (0x00000000U)

/* OBS_PEAKHISTOGRAM19_16_1 */

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM16_1_MASK      (0x000000FFU)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM16_1_SHIFT     (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM16_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM16_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM17_1_MASK      (0x0000FF00U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM17_1_SHIFT     (0x00000008U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM17_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM17_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM18_1_MASK      (0x00FF0000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM18_1_SHIFT     (0x00000010U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM18_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM18_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM19_1_MASK      (0xFF000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM19_1_SHIFT     (0x00000018U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM19_1_RESETVAL  (0x00000000U)
#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_PEAKHISTOGRAM19_1_MAX       (0x000000FFU)

#define CSL_RESOLVER_REGS_OBS_PEAKHISTOGRAM19_16_1_RESETVAL                    (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif