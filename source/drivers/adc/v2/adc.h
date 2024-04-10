/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/**
 *  \defgroup DRV_ADC_MODULE APIs for ADC
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the ADC module.
 *
 *  @{
 */

#ifndef ADC_V2_H_
#define ADC_V2_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Header Files
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_adc.h>
//*****************************************************************************
//
//! Defines used by the driver
//
//*****************************************************************************
//! Register offset difference between 2 ADCSOCxCTL registers
#define ADC_ADCSOCxCTL_STEP (CSL_ADC_ADCSOC1CTL - CSL_ADC_ADCSOC0CTL)
//! Register offset difference between 2 ADCINTSELxNy registers
#define ADC_ADCINTSELxNy_STEP (CSL_ADC_ADCINTSEL3N4 - CSL_ADC_ADCINTSEL1N2)
//! Register offset difference between 2 ADCPPBxCONFIG registers
#define ADC_ADCPPBx_STEP (CSL_ADC_ADCPPB2CONFIG - CSL_ADC_ADCPPB1CONFIG)
//! ADC PPB Trip Mask
#define ADC_ADCPPBTRIP_MASK ((uint32_t)CSL_ADC_ADCPPB1TRIPHI_LIMITHI_MASK \
                               | (uint32_t)CSL_ADC_ADCPPB1TRIPHI_HSIGN_MASK)
//! Register offset difference between 2 ADCPPBxRESULT registers
#define ADC_RESULT_ADCPPBxRESULT_STEP (CSL_ADC_RESULT_ADCPPB2RESULT -\
                                                CSL_ADC_RESULT_ADCPPB1RESULT)
//! Register offset difference between 2 ADCRESULTx registers
#define ADC_RESULT_ADCRESULTx_STEP (CSL_ADC_RESULT_ADCRESULT1 - \
                                        CSL_ADC_RESULT_ADCRESULT0)
//*****************************************************************************
//
// Define to mask out the bits in the REPxCTL register that aren't associated
// with repeater configuration.
//
//*****************************************************************************
#define ADC_REPCTL_MASK             (CSL_ADC_REP1CTL_MODE_MASK               |\
                                     CSL_ADC_REP1CTL_TRIGGER_MASK            |\
                                     CSL_ADC_REP1CTL_SYNCINSEL_MASK)
//! Register offset difference between 2 ADCPPBxLIMIT registers


#define ADC_ADCPPBxCONFIG2_STEP (CSL_ADC_ADCPPB2CONFIG2 - CSL_ADC_ADCPPB1CONFIG2)
#define ADC_REPxCTL_STEP        (CSL_ADC_REP2CTL - CSL_ADC_REP1CTL)
#define ADC_REPxN_STEP          (CSL_ADC_REP2N - CSL_ADC_REP1N)
#define ADC_REPxPHASE_STEP      (CSL_ADC_REP2PHASE - CSL_ADC_REP1PHASE)
#define ADC_REPxSPREAD_STEP     (CSL_ADC_REP2SPREAD - CSL_ADC_REP1SPREAD)


#define ADC_PPBxTRIPHI_STEP     (CSL_ADC_ADCPPB1TRIPHI - CSL_ADC_ADCPPB1TRIPHI)
#define ADC_PPBxTRIPLO_STEP     (CSL_ADC_ADCPPB2TRIPLO - CSL_ADC_ADCPPB1TRIPLO)
#define ADC_ADCPPBxLIMIT_STEP   (CSL_ADC_ADCPPB2LIMIT - CSL_ADC_ADCPPB1LIMIT)
#define ADC_ADCPPBxPCOUNT_STEP   (CSL_ADC_ADCPPBP2PCOUNT - CSL_ADC_ADCPPBP1PCOUNT)
#define ADC_ADCPPBxCONFIG2_STEP   (CSL_ADC_ADCPPB2CONFIG2 - CSL_ADC_ADCPPB1CONFIG2)
#define ADC_ADCPPBxPSUM_STEP   (CSL_ADC_ADCPPB2PSUM - CSL_ADC_ADCPPB1PSUM)
#define ADC_ADCPPBxPMAX_STEP   (CSL_ADC_ADCPPB2PMAX - CSL_ADC_ADCPPB1PMAX)
#define ADC_ADCPPBxPMAXI_STEP   (CSL_ADC_ADCPPB2PMAXI - CSL_ADC_ADCPPB1PMAXI)
#define ADC_ADCPPBxPMIN_STEP   (CSL_ADC_ADCPPB2PMIN - CSL_ADC_ADCPPB1PMIN)
#define ADC_ADCPPBxPMINI_STEP   (CSL_ADC_ADCPPB2PMINI - CSL_ADC_ADCPPB1PMINI)
#define ADC_ADCPPBxTRIPLO2_STEP   (CSL_ADC_ADCPPB2TRIPLO2 - CSL_ADC_ADCPPB1TRIPLO2)

//*****************************************************************************
//
// Define to mask out the bits in the REPSTATUS register that aren't
// associated with trigger repeater block status.
//
//*****************************************************************************
#define ADC_REPSTATUS_MASK      (CSL_ADC_REP1CTL_MODULEBUSY_MASK             |\
                                 CSL_ADC_REP1CTL_PHASEOVF_MASK               |\
                                 CSL_ADC_REP1CTL_TRIGGEROVF_MASK)

//*****************************************************************************
//
// Define to mask out the bits in the CHECKSTATUS register that aren't
// associated with safety checker result status.
//
//*****************************************************************************
#define ADC_SAFECHECK_STATUS_MASK   (CSL_ADC_SAFETY_CHECKSTATUS_RES1READY_MASK|\
                                     CSL_ADC_SAFETY_CHECKSTATUS_RES2READY_MASK|\
                                     CSL_ADC_SAFETY_CHECKSTATUS_OOT_MASK)
//*****************************************************************************
//
// Values that can be passed to ADC_enablePPBEvent(), ADC_disablePPBEvent(),
// ADC_enablePPBEventInterrupt(), ADC_disablePPBEventInterrupt(), and
// ADC_clearPPBEventStatus() as the intFlags and evtFlags parameters. They also
// make up the enumerated bit field returned by ADC_getPPBEventStatus().
//
//*****************************************************************************
#define ADC_EVT_TRIPHI    (0x0001U) //!< Trip High Event
#define ADC_EVT_TRIPLO    (0x0002U) //!< Trip Low Event
#define ADC_EVT_ZERO    (0x0004U) //!< Zero Crossing Event

//*****************************************************************************
//
// Values that can be passed to ADC_forceMultipleSOC() as socMask parameter.
// These values can be OR'd together to trigger multiple SOCs at a time.
//
//*****************************************************************************
#define ADC_FORCE_SOC0    (0x0001U)  //!< SW trigger ADC SOC 0
#define ADC_FORCE_SOC1    (0x0002U)  //!< SW trigger ADC SOC 1
#define ADC_FORCE_SOC2    (0x0004U)  //!< SW trigger ADC SOC 2
#define ADC_FORCE_SOC3    (0x0008U)  //!< SW trigger ADC SOC 3
#define ADC_FORCE_SOC4    (0x0010U)  //!< SW trigger ADC SOC 4
#define ADC_FORCE_SOC5    (0x0020U)  //!< SW trigger ADC SOC 5
#define ADC_FORCE_SOC6    (0x0040U)  //!< SW trigger ADC SOC 6
#define ADC_FORCE_SOC7    (0x0080U)  //!< SW trigger ADC SOC 7
#define ADC_FORCE_SOC8    (0x0100U)  //!< SW trigger ADC SOC 8
#define ADC_FORCE_SOC9    (0x0200U)  //!< SW trigger ADC SOC 9
#define ADC_FORCE_SOC10    (0x0400U)  //!< SW trigger ADC SOC 10
#define ADC_FORCE_SOC11    (0x0800U)  //!< SW trigger ADC SOC 11
#define ADC_FORCE_SOC12    (0x1000U)  //!< SW trigger ADC SOC 12
#define ADC_FORCE_SOC13    (0x2000U)  //!< SW trigger ADC SOC 13
#define ADC_FORCE_SOC14    (0x4000U)  //!< SW trigger ADC SOC 14
#define ADC_FORCE_SOC15    (0x8000U)  //!< SW trigger ADC SOC 15

//*****************************************************************************
//
//! Values that can be passed to ADC_setPrescaler() as the \e clkPrescale
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_CLK_DIV_1_0 = 0,  //!< ADCCLK = (input clock) / 1.0
    ADC_CLK_DIV_2_0 = 2,  //!< ADCCLK = (input clock) / 2.0
    ADC_CLK_DIV_2_5 = 3,  //!< ADCCLK = (input clock) / 2.5
    ADC_CLK_DIV_3_0 = 4,  //!< ADCCLK = (input clock) / 3.0
    ADC_CLK_DIV_3_5 = 5,  //!< ADCCLK = (input clock) / 3.5
    ADC_CLK_DIV_4_0 = 6,  //!< ADCCLK = (input clock) / 4.0
    ADC_CLK_DIV_4_5 = 7,  //!< ADCCLK = (input clock) / 4.5
    ADC_CLK_DIV_5_0 = 8,  //!< ADCCLK = (input clock) / 5.0
    ADC_CLK_DIV_5_5 = 9,  //!< ADCCLK = (input clock) / 5.5
    ADC_CLK_DIV_6_0 = 10, //!< ADCCLK = (input clock) / 6.0
    ADC_CLK_DIV_6_5 = 11,  //!< ADCCLK = (input clock) / 6.5
    ADC_CLK_DIV_7_0 = 12,  //!< ADCCLK = (input clock) / 7.0
    ADC_CLK_DIV_7_5 = 13,  //!< ADCCLK = (input clock) / 7.5
    ADC_CLK_DIV_8_0 = 14,  //!< ADCCLK = (input clock) / 8.0
    ADC_CLK_DIV_8_5 = 15   //!< ADCCLK = (input clock) / 8.5
} ADC_ClkPrescale;

//*****************************************************************************
//
//! Values that can be passed to ADC_setMode() as the \e resolution
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_RESOLUTION_12BIT = 0  //!< 12-bit conversion resolution
} ADC_Resolution;

//*****************************************************************************
//
//! Values that can be passed to ADC_setMode() as the \e signalMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_MODE_SINGLE_ENDED = 0,  //!< Sample on single pin with VREFLO
    ADC_MODE_DIFFERENTIAL = 1   //!< Sample on pair of pins
} ADC_SignalMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_setupSOC() as the \e trigger
//! parameter to specify the event that will trigger a conversion to start.
//! It is also used with ADC_setBurstModeConfig().
//
//*****************************************************************************
typedef enum
{
    ADC_TRIGGER_SW_ONLY = 0x00,  //!< Software only
    ADC_TRIGGER_RTI0 = 0x01,  //!< RTI Timer 0
    ADC_TRIGGER_RTI1 = 0x02,  //!< RTI Timer 1
    ADC_TRIGGER_RTI2 = 0x03,  //!< RTI Timer 2
    ADC_TRIGGER_RTI3 = 0x04,  //!< RTI Timer 3
    ADC_TRIGGER_INPUT_XBAR_OUT5 = 0x05,  //!< InputXBar.Out[5]
    ADC_TRIGGER_EPWM0_SOCA = 0x08,  //!< ePWM0, ADCSOCA
    ADC_TRIGGER_EPWM0_SOCB = 0x09,  //!< ePWM0, ADCSOCB
    ADC_TRIGGER_EPWM1_SOCA = 0x0A,  //!< ePWM1, ADCSOCA
    ADC_TRIGGER_EPWM1_SOCB = 0x0B,  //!< ePWM1, ADCSOCB
    ADC_TRIGGER_EPWM2_SOCA = 0x0C,  //!< ePWM2, ADCSOCA
    ADC_TRIGGER_EPWM2_SOCB = 0x0D,  //!< ePWM2, ADCSOCB
    ADC_TRIGGER_EPWM3_SOCA = 0x0E,  //!< ePWM3, ADCSOCA
    ADC_TRIGGER_EPWM3_SOCB = 0x0F,  //!< ePWM3, ADCSOCB
    ADC_TRIGGER_EPWM4_SOCA = 0x10,  //!< ePWM4, ADCSOCA
    ADC_TRIGGER_EPWM4_SOCB = 0x11,  //!< ePWM4, ADCSOCB
    ADC_TRIGGER_EPWM5_SOCA = 0x12,  //!< ePWM5, ADCSOCA
    ADC_TRIGGER_EPWM5_SOCB = 0x13,  //!< ePWM5, ADCSOCB
    ADC_TRIGGER_EPWM6_SOCA = 0x14,  //!< ePWM6, ADCSOCA
    ADC_TRIGGER_EPWM6_SOCB = 0x15,  //!< ePWM6, ADCSOCB
    ADC_TRIGGER_EPWM7_SOCA = 0x16,  //!< ePWM7, ADCSOCA
    ADC_TRIGGER_EPWM7_SOCB = 0x17,  //!< ePWM7, ADCSOCB
    ADC_TRIGGER_EPWM8_SOCA = 0x18,  //!< ePWM8, ADCSOCA
    ADC_TRIGGER_EPWM8_SOCB = 0x19,  //!< ePWM8, ADCSOCB
    ADC_TRIGGER_EPWM9_SOCA = 0x1A,  //!< ePWM9, ADCSOCA
    ADC_TRIGGER_EPWM9_SOCB = 0x1B,  //!< ePWM9, ADCSOCB
    ADC_TRIGGER_EPWM10_SOCA = 0x1C,  //!< ePWM10, ADCSOCA
    ADC_TRIGGER_EPWM10_SOCB = 0x1D,  //!< ePWM10, ADCSOCB
    ADC_TRIGGER_EPWM11_SOCA = 0x1E,  //!< ePWM11, ADCSOCA
    ADC_TRIGGER_EPWM11_SOCB = 0x1F,  //!< ePWM11, ADCSOCB
    ADC_TRIGGER_EPWM12_SOCA = 0x20,  //!< ePWM12, ADCSOCA
    ADC_TRIGGER_EPWM12_SOCB = 0x21,  //!< ePWM12, ADCSOCB
    ADC_TRIGGER_EPWM13_SOCA = 0x22,  //!< ePWM13, ADCSOCA
    ADC_TRIGGER_EPWM13_SOCB = 0x23,  //!< ePWM13, ADCSOCB
    ADC_TRIGGER_EPWM14_SOCA = 0x24,  //!< ePWM14, ADCSOCA
    ADC_TRIGGER_EPWM14_SOCB = 0x25,  //!< ePWM14, ADCSOCB
    ADC_TRIGGER_EPWM15_SOCA = 0x26,  //!< ePWM15, ADCSOCA
    ADC_TRIGGER_EPWM15_SOCB = 0x27,  //!< ePWM15, ADCSOCB
    ADC_TRIGGER_EPWM16_SOCA = 0x28,  //!< ePWM16, ADCSOCA
    ADC_TRIGGER_EPWM16_SOCB = 0x29,  //!< ePWM16, ADCSOCB
    ADC_TRIGGER_EPWM17_SOCA = 0x2A,  //!< ePWM17, ADCSOCA
    ADC_TRIGGER_EPWM17_SOCB = 0x2B,  //!< ePWM17, ADCSOCB
    ADC_TRIGGER_EPWM18_SOCA = 0x2C,  //!< ePWM18, ADCSOCA
    ADC_TRIGGER_EPWM18_SOCB = 0x2D,  //!< ePWM18, ADCSOCB
    ADC_TRIGGER_EPWM19_SOCA = 0x2E,  //!< ePWM19, ADCSOCA
    ADC_TRIGGER_EPWM19_SOCB = 0x2F,  //!< ePWM19, ADCSOCB
    ADC_TRIGGER_EPWM20_SOCA = 0x30,  //!< ePWM20, ADCSOCA
    ADC_TRIGGER_EPWM20_SOCB = 0x31,  //!< ePWM20, ADCSOCB
    ADC_TRIGGER_EPWM21_SOCA = 0x32,  //!< ePWM21, ADCSOCA
    ADC_TRIGGER_EPWM21_SOCB = 0x33,  //!< ePWM21, ADCSOCB
    ADC_TRIGGER_EPWM22_SOCA = 0x34,  //!< ePWM22, ADCSOCA
    ADC_TRIGGER_EPWM22_SOCB = 0x35,  //!< ePWM22, ADCSOCB
    ADC_TRIGGER_EPWM23_SOCA = 0x36,  //!< ePWM23, ADCSOCA
    ADC_TRIGGER_EPWM23_SOCB = 0x37,  //!< ePWM23, ADCSOCB
    ADC_TRIGGER_EPWM24_SOCA = 0x38,  //!< ePWM24, ADCSOCA
    ADC_TRIGGER_EPWM24_SOCB = 0x39,  //!< ePWM24, ADCSOCB
    ADC_TRIGGER_EPWM25_SOCA = 0x3A,  //!< ePWM25, ADCSOCA
    ADC_TRIGGER_EPWM25_SOCB = 0x3B,  //!< ePWM25, ADCSOCB
    ADC_TRIGGER_EPWM26_SOCA = 0x3C,  //!< ePWM26, ADCSOCA
    ADC_TRIGGER_EPWM26_SOCB = 0x3D,  //!< ePWM26, ADCSOCB
    ADC_TRIGGER_EPWM27_SOCA = 0x3E,  //!< ePWM27, ADCSOCA
    ADC_TRIGGER_EPWM27_SOCB = 0x3F,  //!< ePWM27, ADCSOCB
    ADC_TRIGGER_EPWM28_SOCA = 0x40,  //!< ePWM28, ADCSOCA
    ADC_TRIGGER_EPWM28_SOCB = 0x41,  //!< ePWM28, ADCSOCB
    ADC_TRIGGER_EPWM29_SOCA = 0x42,  //!< ePWM29, ADCSOCA
    ADC_TRIGGER_EPWM29_SOCB = 0x43,  //!< ePWM29, ADCSOCB
    ADC_TRIGGER_EPWM30_SOCA = 0x44,  //!< ePWM30, ADCSOCA
    ADC_TRIGGER_EPWM30_SOCB = 0x45,  //!< ePWM30, ADCSOCB
    ADC_TRIGGER_EPWM31_SOCA = 0x46,  //!< ePWM31, ADCSOCA
    ADC_TRIGGER_EPWM31_SOCB = 0x47,   //!< ePWM31, ADCSOCB
    ADC_TRIGGER_ECAP0_SOCEVT = 0x48,  //!< eCAP0, SOCEVT
    ADC_TRIGGER_ECAP1_SOCEVT = 0x49,  //!< eCAP1, SOCEVT
    ADC_TRIGGER_ECAP2_SOCEVT = 0x4A,  //!< eCAP2, SOCEVT
    ADC_TRIGGER_ECAP3_SOCEVT = 0x4B,  //!< eCAP3, SOCEVT
    ADC_TRIGGER_ECAP4_SOCEVT = 0x4C,  //!< eCAP4, SOCEVT
    ADC_TRIGGER_ECAP5_SOCEVT = 0x4D,  //!< eCAP5, SOCEVT
    ADC_TRIGGER_ECAP6_SOCEVT = 0x4E,  //!< eCAP6, SOCEVT
    ADC_TRIGGER_ECAP7_SOCEVT = 0x4F,  //!< eCAP7, SOCEVT
    ADC_TRIGGER_ECAP8_SOCEVT = 0x50,  //!< eCAP8, SOCEVT
    ADC_TRIGGER_ECAP9_SOCEVT = 0x51,  //!< eCAP9, SOCEVT
    ADC_TRIGGER_ECAP10_SOCEVT = 0x52,  //!< eCAP10, SOCEVT
    ADC_TRIGGER_ECAP11_SOCEVT = 0x53,  //!< eCAP11, SOCEVT
    ADC_TRIGGER_ECAP12_SOCEVT = 0x54,  //!< eCAP12, SOCEVT
    ADC_TRIGGER_ECAP13_SOCEVT = 0x55,  //!< eCAP13, SOCEVT
    ADC_TRIGGER_ECAP14_SOCEVT = 0x56,  //!< eCAP14, SOCEVT
    ADC_TRIGGER_ECAP15_SOCEVT = 0x57,  //!< eCAP15, SOCEVT
    ADC_TRIGGER_RTI4         = 0x58,  //!< RTI Timer 4
    ADC_TRIGGER_RTI5         = 0x59,  //!< RTI Timer 5
    ADC_TRIGGER_RTI6         = 0x5A,  //!< RTI Timer 6
    ADC_TRIGGER_RTI7         = 0x5B,  //!< RTI Timer 7
    ADC_TRIGGER_REPEATER1    = 0x7E,    //!< Repeater 1
    ADC_TRIGGER_REPEATER2    = 0x7F,    //!< Repeater 2
} ADC_Trigger;

//*****************************************************************************
//
//! Values that can be passed to ADC_setupSOC() as the \e channel
//! parameter. This is the input pin on which the signal to be converted is
//! located.
//
//*****************************************************************************
typedef enum
{
    ADC_CH_ADCIN0 = 0,  //!< single-ended, ADCIN0
    ADC_CH_ADCIN1 = 1,  //!< single-ended, ADCIN1
    ADC_CH_ADCIN2 = 2,  //!< single-ended, ADCIN2
    ADC_CH_ADCIN3 = 3,  //!< single-ended, ADCIN3
    ADC_CH_ADCIN4 = 4,  //!< single-ended, ADCIN4
    ADC_CH_ADCIN5 = 5,  //!< single-ended, ADCIN5
    ADC_CH_CAL0 = 6, //!< single-ended, CAL0
    ADC_CH_CAL1 = 7, //!< single-ended, CAL1
    ADC_CH_ADCIN0_ADCIN1 = 0,  //!< differential, ADCIN0 and ADCIN1
    ADC_CH_ADCIN1_ADCIN0 = 1,  //!< differential, ADCIN1 and ADCIN0
    ADC_CH_ADCIN2_ADCIN3 = 2,  //!< differential, ADCIN2 and ADCIN3
    ADC_CH_ADCIN3_ADCIN2 = 3,  //!< differential, ADCIN3 and ADCIN2
    ADC_CH_ADCIN4_ADCIN5 = 4,  //!< differential, ADCIN4 and ADCIN5
    ADC_CH_ADCIN5_ADCIN4 = 5,  //!< differential, ADCIN5 and ADCIN4
    ADC_CH_CAL0_CAL1 = 6, //!< differential, CAL0 and CAL1
} ADC_Channel;

//*****************************************************************************
//
//! Values that can be passed to ADC_setInterruptPulseMode() as the
//! \e pulseMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Occurs at the end of the acquisition window
    ADC_PULSE_END_OF_ACQ_WIN = 0,
    //! Occurs at the end of the conversion
    ADC_PULSE_END_OF_CONV = 1
} ADC_PulseMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_enableInterrupt(), ADC_disableInterrupt(),
//! ADC_clearInterruptStatus(), ADC_getInterruptOverflowStatus(),
//! ADC_clearInterruptOverflowStatus(), ADC_setInterruptSource(),
//! ADC_enableContinuousMode(), ADC_disableContinuousMode()
//! and ADC_getInterruptStatus() as the \e adcIntNum parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_INT_NUMBER1 = 0,  //!< ADCINT1 Interrupt
    ADC_INT_NUMBER2 = 1,  //!< ADCINT2 Interrupt
    ADC_INT_NUMBER3 = 2,  //!< ADCINT3 Interrupt
    ADC_INT_NUMBER4 = 3   //!< ADCINT4 Interrupt
} ADC_IntNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e ppbNumber parameter for several
//! functions.
//
//*****************************************************************************
typedef enum
{
    ADC_PPB_NUMBER1 = 0,  //!< Post-processing block 1
    ADC_PPB_NUMBER2 = 1,  //!< Post-processing block 2
    ADC_PPB_NUMBER3 = 2,  //!< Post-processing block 3
    ADC_PPB_NUMBER4 = 3   //!< Post-processing block 4
} ADC_PPBNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e socNumber parameter for several
//! functions. This value identifies the start-of-conversion (SOC) that a
//! function is configuring or accessing. Note that in some cases (for example,
//! ADC_setInterruptSource()) \e socNumber is used to refer to the
//! corresponding end-of-conversion (EOC).
//
//*****************************************************************************
typedef enum
{
    ADC_SOC_NUMBER0 = 0,  //!< SOC/EOC number 0
    ADC_SOC_NUMBER1 = 1,  //!< SOC/EOC number 1
    ADC_SOC_NUMBER2 = 2,  //!< SOC/EOC number 2
    ADC_SOC_NUMBER3 = 3,  //!< SOC/EOC number 3
    ADC_SOC_NUMBER4 = 4,  //!< SOC/EOC number 4
    ADC_SOC_NUMBER5 = 5,  //!< SOC/EOC number 5
    ADC_SOC_NUMBER6 = 6,  //!< SOC/EOC number 6
    ADC_SOC_NUMBER7 = 7,  //!< SOC/EOC number 7
    ADC_SOC_NUMBER8 = 8,  //!< SOC/EOC number 8
    ADC_SOC_NUMBER9 = 9,  //!< SOC/EOC number 9
    ADC_SOC_NUMBER10 = 10,  //!< SOC/EOC number 10
    ADC_SOC_NUMBER11 = 11,  //!< SOC/EOC number 11
    ADC_SOC_NUMBER12 = 12,  //!< SOC/EOC number 12
    ADC_SOC_NUMBER13 = 13,  //!< SOC/EOC number 13
    ADC_SOC_NUMBER14 = 14,  //!< SOC/EOC number 14
    ADC_SOC_NUMBER15 = 15   //!< SOC/EOC number 15
} ADC_SOCNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e trigger parameter for the
//! ADC_setInterruptSOCTrigger() function.
//
//*****************************************************************************
typedef enum
{
    ADC_INT_SOC_TRIGGER_NONE = 0,  //!< No ADCINT will trigger the SOC
    ADC_INT_SOC_TRIGGER_ADCINT1 = 1,  //!< ADCINT1 will trigger the SOC
    ADC_INT_SOC_TRIGGER_ADCINT2 = 2   //!< ADCINT2 will trigger the SOC
} ADC_IntSOCTrigger;

//*****************************************************************************
//
//! Values that can be passed to ADC_setSOCPriority() as the \e priMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_PRI_ALL_ROUND_ROBIN = 0,  //!< Round robin mode is used for all
    ADC_PRI_SOC0_HIPRI = 1,  //!< SOC 0 hi pri, others in round robin
    ADC_PRI_THRU_SOC1_HIPRI = 2,  //!< SOC 0-1 hi pri, others in round robin
    ADC_PRI_THRU_SOC2_HIPRI = 3,  //!< SOC 0-2 hi pri, others in round robin
    ADC_PRI_THRU_SOC3_HIPRI = 4,  //!< SOC 0-3 hi pri, others in round robin
    ADC_PRI_THRU_SOC4_HIPRI = 5,  //!< SOC 0-4 hi pri, others in round robin
    ADC_PRI_THRU_SOC5_HIPRI = 6,  //!< SOC 0-5 hi pri, others in round robin
    ADC_PRI_THRU_SOC6_HIPRI = 7,  //!< SOC 0-6 hi pri, others in round robin
    ADC_PRI_THRU_SOC7_HIPRI = 8,  //!< SOC 0-7 hi pri, others in round robin
    ADC_PRI_THRU_SOC8_HIPRI = 9,  //!< SOC 0-8 hi pri, others in round robin
    ADC_PRI_THRU_SOC9_HIPRI = 10,  //!< SOC 0-9 hi pri, others in round robin
    ADC_PRI_THRU_SOC10_HIPRI = 11,  //!< SOC 0-10 hi pri, others in round robin
    ADC_PRI_THRU_SOC11_HIPRI = 12,  //!< SOC 0-11 hi pri, others in round robin
    ADC_PRI_THRU_SOC12_HIPRI = 13,  //!< SOC 0-12 hi pri, others in round robin
    ADC_PRI_THRU_SOC13_HIPRI = 14,  //!< SOC 0-13 hi pri, others in round robin
    ADC_PRI_THRU_SOC14_HIPRI = 15,  //!< SOC 0-14 hi pri, SOC15 in round robin
    ADC_PRI_ALL_HIPRI = 16  //!< All priorities based on SOC number
} ADC_PriorityMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_configOSDetectMode() as the \e modeVal
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_OSDETECT_MODE_DISABLED            = 0x0U,//!< Open/Shorts detection cir-
                                                 //!< cuit(O/S DC) is disabled
    ADC_OSDETECT_MODE_VSSA                = 0x1U,//!< O/S DC is enabled at zero
                                                 //!< scale
    ADC_OSDETECT_MODE_VDDA                = 0x2U,//!< O/S DC is enabled at full
                                                 //!< scale
    ADC_OSDETECT_MODE_5BY12_VDDA          = 0x3U,//!< O/S DC is enabled at 5/12
                                                 //!< scale
    ADC_OSDETECT_MODE_7BY12_VDDA          = 0x4U,//!< O/S DC is enabled at 7/12
                                                 //!< scale
    ADC_OSDETECT_MODE_5K_PULLDOWN_TO_VSSA = 0x5U,//!< O/S DC is enabled at 5K
                                                 //!< pulldown to VSSA
    ADC_OSDETECT_MODE_5K_PULLUP_TO_VDDA   = 0x6U,//!< O/S DC is enabled at 5K
                                                 //!< pullup to VDDA
    ADC_OSDETECT_MODE_7K_PULLDOWN_TO_VSSA = 0x7U //!< O/S DC is enabled at 7K
                                                 //!< pulldown to VSSA
} ADC_OSDetectMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_selectOffsetTrimMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_OFFSET_TRIM_COMMON     = 0x0000U, //!< Common Trim register for all
                                          //!< ADC modes
    ADC_OFFSET_TRIM_INDIVIDUAL = 0x0100U  //!< Individual Trim registers for
                                          //!< different ADC modes
} ADC_OffsetTrim;

typedef enum
{
    ADC_SYNCIN_DISABLE          = 0x00,  //!< ADC Syncin is disabled
    ADC_SYNCIN_EPWM0SYNCOUT     = 0x01,  //!< ADC Syncin is EPWM0SYNCOUT
    ADC_SYNCIN_EPWM1SYNCOUT     = 0x02,  //!< ADC Syncin is EPWM1SYNCOUT
    ADC_SYNCIN_EPWM2SYNCOUT     = 0x03,  //!< ADC Syncin is EPWM2SYNCOUT
    ADC_SYNCIN_EPWM3SYNCOUT     = 0x04,  //!< ADC Syncin is EPWM3SYNCOUT
    ADC_SYNCIN_EPWM4SYNCOUT     = 0x05,  //!< ADC Syncin is EPWM4SYNCOUT
    ADC_SYNCIN_EPWM5SYNCOUT     = 0x06,  //!< ADC Syncin is EPWM5SYNCOUT
    ADC_SYNCIN_EPWM6SYNCOUT     = 0x07,  //!< ADC Syncin is EPWM6SYNCOUT
    ADC_SYNCIN_EPWM7SYNCOUT     = 0x08,  //!< ADC Syncin is EPWM7SYNCOUT
    ADC_SYNCIN_EPWM8SYNCOUT     = 0x09,  //!< ADC Syncin is EPWM8SYNCOUT
    ADC_SYNCIN_EPWM9SYNCOUT     = 0x0A,  //!< ADC Syncin is EPWM9SYNCOUT
    ADC_SYNCIN_EPWM10SYNCOUT    = 0x0B,  //!< ADC Syncin is EPWM10SYNCOUT
    ADC_SYNCIN_EPWM11SYNCOUT    = 0x0C,  //!< ADC Syncin is EPWM11SYNCOUT
    ADC_SYNCIN_EPWM12SYNCOUT    = 0x0D,  //!< ADC Syncin is EPWM12SYNCOUT
    ADC_SYNCIN_EPWM13SYNCOUT    = 0x0E,  //!< ADC Syncin is EPWM13SYNCOUT
    ADC_SYNCIN_EPWM14SYNCOUT    = 0x0F,  //!< ADC Syncin is EPWM14SYNCOUT
    ADC_SYNCIN_EPWM15SYNCOUT    = 0x10,  //!< ADC Syncin is EPWM15SYNCOUT
    ADC_SYNCIN_EPWM16SYNCOUT    = 0x11,  //!< ADC Syncin is EPWM16SYNCOUT
    ADC_SYNCIN_EPWM17SYNCOUT    = 0x12,  //!< ADC Syncin is EPWM17SYNCOUT
    ADC_SYNCIN_EPWM18SYNCOUT    = 0x13,  //!< ADC Syncin is EPWM18SYNCOUT
    ADC_SYNCIN_EPWM19SYNCOUT    = 0x14,  //!< ADC Syncin is EPWM19SYNCOUT
    ADC_SYNCIN_EPWM20SYNCOUT    = 0x15,  //!< ADC Syncin is EPWM20SYNCOUT
    ADC_SYNCIN_EPWM21SYNCOUT    = 0x16,  //!< ADC Syncin is EPWM21SYNCOUT
    ADC_SYNCIN_EPWM22SYNCOUT    = 0x17,  //!< ADC Syncin is EPWM22SYNCOUT
    ADC_SYNCIN_EPWM23SYNCOUT    = 0x18,  //!< ADC Syncin is EPWM23SYNCOUT
    ADC_SYNCIN_EPWM24SYNCOUT    = 0x19,  //!< ADC Syncin is EPWM24SYNCOUT
    ADC_SYNCIN_EPWM25SYNCOUT    = 0x1A,  //!< ADC Syncin is EPWM25SYNCOUT
    ADC_SYNCIN_EPWM26SYNCOUT    = 0x1B,  //!< ADC Syncin is EPWM26SYNCOUT
    ADC_SYNCIN_EPWM27SYNCOUT    = 0x1C,  //!< ADC Syncin is EPWM27SYNCOUT
    ADC_SYNCIN_EPWM28SYNCOUT    = 0x1D,  //!< ADC Syncin is EPWM28SYNCOUT
    ADC_SYNCIN_EPWM29SYNCOUT    = 0x1E,  //!< ADC Syncin is EPWM29SYNCOUT
    ADC_SYNCIN_EPWM30SYNCOUT    = 0x1F,  //!< ADC Syncin is EPWM30SYNCOUT
    ADC_SYNCIN_EPWM31SYNCOUT    = 0x20,  //!< ADC Syncin is EPWM31SYNCOUT
    ADC_SYNCIN_ECAP0SYNCOUT     = 0x21,  //!< ADC Syncin is ECAP0SYNCOUT
    ADC_SYNCIN_ECAP1SYNCOUT     = 0x22,  //!< ADC Syncin is ECAP1SYNCOUT
    ADC_SYNCIN_ECAP2SYNCOUT     = 0x23,  //!< ADC Syncin is ECAP2SYNCOUT
    ADC_SYNCIN_ECAP3SYNCOUT     = 0x24,  //!< ADC Syncin is ECAP3SYNCOUT
    ADC_SYNCIN_ECAP4SYNCOUT     = 0x25,  //!< ADC Syncin is ECAP4SYNCOUT
    ADC_SYNCIN_ECAP5SYNCOUT     = 0x26,  //!< ADC Syncin is ECAP5SYNCOUT
    ADC_SYNCIN_ECAP6SYNCOUT     = 0x27,  //!< ADC Syncin is ECAP6SYNCOUT
    ADC_SYNCIN_ECAP7SYNCOUT     = 0x28,  //!< ADC Syncin is ECAP7SYNCOUT
    ADC_SYNCIN_ECAP8SYNCOUT     = 0x29,  //!< ADC Syncin is ECAP8SYNCOUT
    ADC_SYNCIN_ECAP9SYNCOUT     = 0x2A,  //!< ADC Syncin is ECAP9SYNCOUT
    ADC_SYNCIN_ECAP10SYNCOUT    = 0x2B,  //!< ADC Syncin is ECAP10SYNCOUT
    ADC_SYNCIN_ECAP11SYNCOUT    = 0x2C,  //!< ADC Syncin is ECAP11SYNCOUT
    ADC_SYNCIN_ECAP12SYNCOUT    = 0x2D,  //!< ADC Syncin is ECAP12SYNCOUT
    ADC_SYNCIN_ECAP13SYNCOUT    = 0x2E,  //!< ADC Syncin is ECAP13SYNCOUT
    ADC_SYNCIN_ECAP14SYNCOUT    = 0x2F,  //!< ADC Syncin is ECAP14SYNCOUT
    ADC_SYNCIN_ECAP15SYNCOUT    = 0x30,  //!< ADC Syncin is ECAP15SYNCOUT
    ADC_SYNCIN_INPUTXBAROUTPUT6 = 0x31,  //!< ADC Syncin is INPUTXBAROUTPUT6
    ADC_SYNCIN_INPUTXBAROUTPUT7 = 0x32,  //!< ADC Syncin is INPUTXBAROUTPUT7
    ADC_SYNCIN_CPSW_CTPS_SYNC    = 0x33,  //!< ADC Syncin is CPSW_CTPS_SYNC
} ADC_SyncInput;

//*****************************************************************************
//
//! Values that can be passed to ADC_selectPPBOSINTSource() as the \e osIntSrc
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_PPB_OS_INT_1 = 0x0U,          //!< PCount generates PPB interrupt
    ADC_PPB_OS_INT_2 = 0x1U           //!< PCount/Sync generates PPB interrupt
} ADC_PPBIntSrcSelect;

//*****************************************************************************
//
//! Values that can be passed to ADC_selectSOCExtChannel() as the \e extChannel
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_CH_ADCINX_0  = 0x0,          //!< ADCINX.0 is converted
    ADC_CH_ADCINX_1  = 0x1,          //!< ADCINX.1 is converted
    ADC_CH_ADCINX_2  = 0x2,          //!< ADCINX.2 is converted
    ADC_CH_ADCINX_3  = 0x3,          //!< ADCINX.3 is converted
} ADC_ExtChannel;

//*****************************************************************************
//
//! Values that can be passed to ADC_setInterruptSource() as the \e intTrigger
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_INT_TRIGGER_EOC0   = 0,        //!< SOC/EOC0
    ADC_INT_TRIGGER_EOC1   = 1,        //!< SOC/EOC1
    ADC_INT_TRIGGER_EOC2   = 2,        //!< SOC/EOC2
    ADC_INT_TRIGGER_EOC3   = 3,        //!< SOC/EOC3
    ADC_INT_TRIGGER_EOC4   = 4,        //!< SOC/EOC4
    ADC_INT_TRIGGER_EOC5   = 5,        //!< SOC/EOC5
    ADC_INT_TRIGGER_EOC6   = 6,        //!< SOC/EOC6
    ADC_INT_TRIGGER_EOC7   = 7,        //!< SOC/EOC7
    ADC_INT_TRIGGER_EOC8   = 8,        //!< SOC/EOC8
    ADC_INT_TRIGGER_EOC9   = 9,        //!< SOC/EOC9
    ADC_INT_TRIGGER_EOC10  = 10,       //!< SOC/EOC10
    ADC_INT_TRIGGER_EOC11  = 11,       //!< SOC/EOC11
    ADC_INT_TRIGGER_EOC12  = 12,       //!< SOC/EOC12
    ADC_INT_TRIGGER_EOC13  = 13,       //!< SOC/EOC13
    ADC_INT_TRIGGER_EOC14  = 14,       //!< SOC/EOC14
    ADC_INT_TRIGGER_EOC15  = 15,       //!< SOC/EOC15
    ADC_INT_TRIGGER_OSINT1 = 16,       //!< OSINT1
    ADC_INT_TRIGGER_OSINT2 = 17,       //!< OSINT2
    ADC_INT_TRIGGER_OSINT3 = 18,       //!< OSINT3
    ADC_INT_TRIGGER_OSINT4 = 19        //!< OSINT4
} ADC_IntTrigger;

//*****************************************************************************
//
//! Values that can be passed to ADC_selectPPBCompareSource() as the \e compSrc
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_PPB_COMPSOURCE_RESULT = 0x0,   //!< PPB compare source is ADCRESULT
    ADC_PPB_COMPSOURCE_PSUM   = 0x1,   //!< PPB compare source is PSUM
    ADC_PPB_COMPSOURCE_SUM    = 0x2    //!< PPB compare source is SUM
} ADC_PPBCompSource;

//*****************************************************************************
//
//! Values that can be passed to ADC_configureSafetyChecker() as the
//! \e adcInst parameter.
//
//*****************************************************************************
typedef enum
{
     ADC_0 = 0,                        //!< Select ADC0 instance
     ADC_1 = 1,                        //!< Select ADC1 instance
     ADC_2 = 2,                         //!< Select ADC2 instance
     ADC_3 = 3,                         //!< Select ADC3 instance
     ADC_4 = 4                         //!< Select ADC4 instance
} ADC_Select;

//*****************************************************************************
//
//! Values that can be passed to ADC_configureSafetyChecker() as the
//! \e adcResultInst parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_RESULT0  = 0,                  //!< Select ADC Result 0
    ADC_RESULT1  = 1,                  //!< Select ADC Result 1
    ADC_RESULT2  = 2,                  //!< Select ADC Result 2
    ADC_RESULT3  = 3,                  //!< Select ADC Result 3
    ADC_RESULT4  = 4,                  //!< Select ADC Result 4
    ADC_RESULT5  = 5,                  //!< Select ADC Result 5
    ADC_RESULT6  = 6,                  //!< Select ADC Result 6
    ADC_RESULT7  = 7,                  //!< Select ADC Result 7
    ADC_RESULT8  = 8,                  //!< Select ADC Result 8
    ADC_RESULT9  = 9,                  //!< Select ADC Result 9
    ADC_RESULT10 = 10,                 //!< Select ADC Result 10
    ADC_RESULT11 = 11,                 //!< Select ADC Result 11
    ADC_RESULT12 = 12,                 //!< Select ADC Result 12
    ADC_RESULT13 = 13,                 //!< Select ADC Result 13
    ADC_RESULT14 = 14,                 //!< Select ADC Result 14
    ADC_RESULT15 = 15                  //!< Select ADC Result 15
} ADC_ResultSelect;

//*****************************************************************************
//
//! Values that can be passed to ADC_configSOCSafetyCheckerInput() as the
//! \e scInput parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_SAFETY_CHECKER_INPUT_DISABLE  = 0x0, //!< Safety checker i/p disabled
    ADC_SAFETY_CHECKER_INPUT_SOCx     = 0x1, //!< Safety checker i/p is SOCx
    ADC_SAFETY_CHECKER_INPUT_PPBx     = 0x2, //!< Safety checker i/p is PPBx
    ADC_SAFETY_CHECKER_INPUT_PPBSUMx  = 0x3  //!< Safety checker i/p is PPBSUMx
} ADC_SafetyCheckerInput;

//*****************************************************************************
//
//! Values that can be passed to ADC_getSafetyCheckerResult() as
//! \e checkInst parameter.
//
//*****************************************************************************
typedef enum
{
  ADC_SAFETY_CHECK1  = 0x0,            //!< Safety Check Result 1
  ADC_SAFETY_CHECK2  = 0x4             //!< Safety Check Result 2
} ADC_SafetyCheckInst;

//*****************************************************************************
//
//! Values that can be passed to ADC_enableSafetyCheckEvt() and
//! ADC_disableSafetyCheckEvt() as \e checkEvent parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_SAFETY_CHECK_EVENT1 = 0,       //!< Safety Check Event 1
    ADC_SAFETY_CHECK_EVENT2 = 16,       //!< Safety Check Event 2
    ADC_SAFETY_CHECK_EVENT3 = 32,      //!< Safety Check Event 3
    ADC_SAFETY_CHECK_EVENT4 = 48       //!< Safety Check Event 4
} ADC_SafetyCheckEvent;

//*****************************************************************************
//
//! Values that can be passed to ADC_enableSafetyCheckEvt(),
//! ADC_disableSafetyCheckEvt(), ADC_enableSafetyCheckInt() and
//! ADC_disableSafetyCheckInt() as \e checkResult parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_SAFETY_CHECK_RES1OVF = 0,      //!< Safety Check Result1 Overflow
    ADC_SAFETY_CHECK_RES2OVF = 4,      //!< Safety Check Result2 Overflow
    ADC_SAFETY_CHECK_OOT     = 8       //!< Safety Check OOT
} ADC_SafetyCheckResult;

//*****************************************************************************
//
//! Values that can be passed to ADC_enableSafetyCheckEvt(),
//! ADC_disableSafetyCheckEvt(), ADC_enableSafetyCheckInt(),
//! ADC_disableSafetyCheckInt(), ADC_getSafetyCheckStatus(),
//! ADC_clearSafetyCheckStatus(), ADC_getSafetyCheckIntStatus() and
//! ADC_clearSafetyCheckIntStatus() as \e checkerNumber parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_SAFETY_CHECKER1 = 0,           //!< Safety Checker1
    ADC_SAFETY_CHECKER2 = 1,           //!< Safety Checker2
    ADC_SAFETY_CHECKER3 = 2,           //!< Safety Checker3
    ADC_SAFETY_CHECKER4 = 3,           //!< Safety Checker4
    ADC_SAFETY_CHECKER5 = 4,           //!< Safety Checker5
    ADC_SAFETY_CHECKER6 = 5,           //!< Safety Checker6
    ADC_SAFETY_CHECKER7 = 6,           //!< Safety Checker7
    ADC_SAFETY_CHECKER8 = 7,            //!< Safety Checker8
    ADC_SAFETY_CHECKER9 = 8,            //!< Safety Checker9
    ADC_SAFETY_CHECKER10 = 9,            //!< Safety Checker10
    ADC_SAFETY_CHECKER11 = 10,            //!< Safety Checker11
    ADC_SAFETY_CHECKER12 = 11            //!< Safety Checker12
}ADC_Checker;

//*****************************************************************************
//
//! Values that can be passed to ADC_getSafetyCheckStatus() and
//! ADC_clearSafetyCheckStatus(), as \e checkerFlag parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_SAFETY_CHECK_OOT_FLG     = 0,  //!< Safety Check Out-of-Tolerance Flag
    ADC_SAFETY_CHECK_RES1OVF_FLG = 4,  //!< Safety Check Result1 Overflow Flag
    ADC_SAFETY_CHECK_RES2OVF_FLG = 8   //!< Safety Check Result2 Overflow Flag
}ADC_SafetyCheckFlag;

//*****************************************************************************
//
//! Values that can be passed to ADC_configureRepeater() as the \e repInstance
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_REPINST1 = 0x0,                //!< Select ADC repeater instance 1
    ADC_REPINST2 = 0x1                 //!< Select ADC repeater instance 2
} ADC_RepInstance;

//*****************************************************************************
//
//! Values that can be passed to ADC_configureRepeater() as the
//! \e config.repMode and ADC_triggerRepeaterMode() as \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_REPMODE_OVERSAMPLING   = 0x0,  //!< ADC repeater mode is oversampling
    ADC_REPMODE_UNDERSAMPLING  = 0x1   //!< ADC repeater mode is undersampling
} ADC_RepMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_configureRepeater() as the
//! \e config parameter.
//
//*****************************************************************************
typedef struct
{
    ADC_RepMode repMode;     //!< Repeater Mode
    ADC_Trigger repTrigger;  //!< Repeater Trigger
    ADC_SyncInput repSyncin; //!< Repeater Syncin
    uint16_t repCount;       //!< Repeater trigger count
    uint16_t repPhase;       //!< Repeater trigger phase delay in sysclk cycles
    uint16_t repSpread;      //!< Repeater trigger spread in sysclk cycles
} ADC_RepeaterConfig;

//*****************************************************************************
//
//! Defines used by the driver
//
//*****************************************************************************
//! Register offset difference between 2 ADCSOCxCTL registers
#define ADC_ADCSOCxCTL_STEP (CSL_ADC_ADCSOC1CTL - CSL_ADC_ADCSOC0CTL)
//! Register offset difference between 2 ADCINTSELxNy registers
#define ADC_ADCINTSELxNy_STEP (CSL_ADC_ADCINTSEL3N4 - CSL_ADC_ADCINTSEL1N2)
//! Register offset difference between 2 ADCPPBxCONFIG registers
#define ADC_ADCPPBx_STEP (CSL_ADC_ADCPPB2CONFIG - CSL_ADC_ADCPPB1CONFIG)
//! ADC PPB Trip Mask
#define ADC_ADCPPBTRIP_MASK ((uint32_t)CSL_ADC_ADCPPB1TRIPHI_LIMITHI_MASK \
                               | (uint32_t)CSL_ADC_ADCPPB1TRIPHI_HSIGN_MASK)
//! Register offset difference between 2 ADCPPBxRESULT registers
#define ADC_RESULT_ADCPPBxRESULT_STEP (CSL_ADC_RESULT_ADCPPB2RESULT -\
                                                CSL_ADC_RESULT_ADCPPB1RESULT)
//! Register offset difference between 2 ADCRESULTx registers
#define ADC_RESULT_ADCRESULTx_STEP (CSL_ADC_RESULT_ADCRESULT1 - \
                                        CSL_ADC_RESULT_ADCRESULT0)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! Configures the analog-to-digital converter module prescaler.
//!
//! \param base is the base address of the ADC module.
//! \param clkPrescale is the ADC clock prescaler.
//!
//! This function configures the ADC module's ADCCLK.
//!
//! The \e clkPrescale parameter specifies the value by which the input clock
//! is divided to make the ADCCLK.  The clkPrescale value can be specified with
//! any of the following variables:
//! \b ADC_CLK_DIV_1_0, \b ADC_CLK_DIV_2_0, \b ADC_CLK_DIV_2_5, ...,
//! \b ADC_CLK_DIV_7_5, \b ADC_CLK_DIV_8_0, or \b ADC_CLK_DIV_8_5.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setPrescaler(uint32_t base, ADC_ClkPrescale clkPrescale)
{
    //
    // Set the configuration of the ADC module prescaler.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL2,
        ((HW_RD_REG16(base + CSL_ADC_ADCCTL2) &
        ~CSL_ADC_ADCCTL2_PRESCALE_MASK) | (uint16_t)clkPrescale));
}

//*****************************************************************************
//
//! Configures a start-of-conversion (SOC) in the ADC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param trigger the source that will cause the SOC.
//! \param channel is the number associated with the input signal.
//! \param sampleWindow is the acquisition window duration.
//!
//! This function configures the a start-of-conversion (SOC) in the ADC module.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e trigger specifies the event that causes the SOC such as software, a
//! timer interrupt, an ePWM event, or an ADC interrupt. It should be a value
//! in the format of \b ADC_TRIGGER_XXXX where XXXX is the event such as
//! \b ADC_TRIGGER_SW_ONLY, \b ADC_TRIGGER_EPWM1_SOCA, and so on.
//!
//! The \e channel parameter specifies the channel to be converted. In
//! single-ended mode this is a single pin given by \b ADC_CH_ADCINx where x is
//! the number identifying the pin between 0 and 5 inclusive. In differential
//! mode, two pins are used as inputs and are passed in the \e channel
//! parameter as \b ADC_CH_ADCIN0_ADCIN1, \b ADC_CH_ADCIN2_ADCIN3, ..., or
//! \b ADC_CH_ADCIN14_ADCIN5.
//!
//! The \e sampleWindow parameter is the acquisition window duration in SYSCLK
//! cycles. It should be a value between 1 and 512 cycles inclusive. The
//! selected duration must be at least as long as one ADCCLK cycle. Also, the
//! datasheet will specify a minimum window duration requirement in
//! nanoseconds.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setupSOC(uint32_t base, ADC_SOCNumber socNumber, ADC_Trigger trigger,
             ADC_Channel channel, uint32_t sampleWindow)
{
    uint32_t ctlRegAddr;

    //
    // Check the arguments.
    //
    DebugP_assert((sampleWindow >= 16U) && (sampleWindow <= 512U));

    //
    // Calculate address for the SOC control register.
    //
    ctlRegAddr = base + CSL_ADC_ADCSOC0CTL +
        ((uint32_t)socNumber * ADC_ADCSOCxCTL_STEP);

    //
    // Set the configuration of the specified SOC.
    //
    HW_WR_REG32(ctlRegAddr,
        (((uint32_t)channel << CSL_ADC_ADCSOC0CTL_CHSEL_SHIFT) |
        ((uint32_t)trigger << CSL_ADC_ADCSOC0CTL_TRIGSEL_SHIFT) |
        (sampleWindow - 1U)));
}

//*****************************************************************************
//
//! Configures the external channel mux for an ADC SOC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param extChannel is the desired external channel.
//!
//! This function configures the external channel for an SOC by configuring
//! the external channel mux for an SOC.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e extChannel is the desired external channel. Valid values can be
//! refered from the enum \e ADC_ExtChannel.
//!
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_selectSOCExtChannel(uint32_t base, ADC_SOCNumber socNumber,
                        uint16_t extChannel)
{
    uint32_t ctlRegAddr;

    //
    // Check the arguments.
    //
    DebugP_assert(extChannel <= 3U);

    //
    // Calculate address for the SOC control register.
    //
    ctlRegAddr = base + CSL_ADC_ADCSOC0CTL +
        ((uint32_t)socNumber * ADC_ADCSOCxCTL_STEP);

    //
    // Set the external channel configuration of the specified SOC.
    //
    HW_WR_REG32(ctlRegAddr,
        ((HW_RD_REG32(ctlRegAddr) & ~((uint32_t)CSL_ADC_ADCSOC0CTL_EXTCHSEL_MASK)) |
        (uint32_t)extChannel));

}

//*****************************************************************************
//
//! Forces software trigger to ADC trigger repeater block
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance to be triggered.
//!
//! This function forces the selected ADC repeater block.
//!
//! Valid values for \e repInstance parameter can be any of the individual
//! ADC_REPEATER_INSTANCEx values defined by enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceRepeaterTrigger(uint32_t base, uint16_t repInstance)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Triggers the selected repeater instance
    //
    HW_WR_REG16(regOffset + CSL_ADC_REP1FRC,
        ((HW_RD_REG16(regOffset + CSL_ADC_REP1FRC) |
        CSL_ADC_REP1FRC_SWFRC_MASK)));

}

//*****************************************************************************
//
//! Gets the current status for repeater block.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance
//!
//! This function returns the current status for the repeater block.
//!
//! \return Returns the current event status, enumerated as a bit field of
//! \b CSL_ADC_REP1CTL_MODULEBUSY_MASK, \b CSL_ADC_REP1CTL_PHASEOVF_MASK, and
//! \b CSL_ADC_REP1CTL_TRIGGEROVF_MASK.
//
//*****************************************************************************
static inline uint16_t
ADC_getRepeaterStatus(uint32_t base, uint16_t repInstance)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Return the status of repeater.
    //
    return(HW_RD_REG16(regOffset + CSL_ADC_REP1CTL) & ADC_REPSTATUS_MASK);
}

//*****************************************************************************
//
//! Configures the interrupt SOC trigger of an SOC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param trigger the interrupt source that will cause the SOC.
//!
//! This function configures the interrupt start-of-conversion trigger in
//! the ADC module.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e trigger specifies the interrupt that causes a start of conversion or
//! none. It should be one of the following values.
//!
//! - \b ADC_INT_SOC_TRIGGER_NONE
//! - \b ADC_INT_SOC_TRIGGER_ADCINT1
//! - \b ADC_INT_SOC_TRIGGER_ADCINT2
//!
//! This functionality is useful for creating continuous conversions.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptSOCTrigger(uint32_t base, ADC_SOCNumber socNumber,
                           ADC_IntSOCTrigger trigger)
{
    uint16_t shiftVal;

    //
    // Each SOC has a 2-bit field in this register.
    //
    shiftVal = (uint16_t)socNumber << 1U;

    //
    // Set the configuration of the specified SOC. Note that we're treating
    // ADCINTSOCSEL1 and ADCINTSOCSEL2 as one 32-bit register here.
    //
    HW_WR_REG32(base + CSL_ADC_ADCINTSOCSEL1,
        ((HW_RD_REG32(base + CSL_ADC_ADCINTSOCSEL1) &
        ~((uint32_t)CSL_ADC_ADCINTSOCSEL1_SOC0_MASK << shiftVal)) |
        ((uint32_t)trigger << shiftVal)));
}

//*****************************************************************************
//
//! Sets the timing of the end-of-conversion pulse
//!
//! \param base is the base address of the ADC module.
//! \param pulseMode is the generation mode of the EOC pulse.
//!
//! This function configures the end-of-conversion (EOC) pulse generated by ADC.
//! This pulse will be generated either at the end of the acquisition window
//! plus a number of SYSCLK cycles configured by ADC_setInterruptCycleOffset()
//! (pass \b ADC_PULSE_END_OF_ACQ_WIN into \e pulseMode) or at the end of the
//! voltage conversion, one cycle prior to the ADC result latching into it's
//! result register (pass \b ADC_PULSE_END_OF_CONV into \e pulseMode).
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptPulseMode(uint32_t base, ADC_PulseMode pulseMode)
{
    //
    // Set the position of the pulse.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        ((HW_RD_REG16(base + CSL_ADC_ADCCTL1) &
        ~CSL_ADC_ADCCTL1_INTPULSEPOS_MASK) |
        ((uint16_t)pulseMode<<CSL_ADC_ADCCTL1_INTPULSEPOS_SHIFT)));
}

//*****************************************************************************
//
//! Sets the timing of early interrupt generation.
//!
//! \param base is the base address of the ADC module.
//! \param cycleOffset is the cycles from an SOC falling edge to an early
//! interrupt pulse.
//!
//! This function configures cycle offset between the negative edge of a sample
//! pulse and an early interrupt pulse being generated. This number of cycles
//! is specified with the \e cycleOffset parameter.
//!
//! This function only applies when early interrupt generation is enabled. That
//! means the ADC_setInterruptPulseMode() function \e pulseMode parameter is
//! configured as \b ADC_PULSE_END_OF_ACQ_WIN.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptCycleOffset(uint32_t base, uint16_t cycleOffset)
{
    //
    // Set the position of the pulse.
    //
    HW_WR_REG16(base + CSL_ADC_ADCINTCYCLE, cycleOffset);
}

//*****************************************************************************
//
//! Enables alternate timings for DMA trigger
//!
//! \param base is the base address of the ADC module.
//!
//! This function enables the alternate timings(tDMA) for DMA trigger. When
//! enabled the DMA is always triggered at tDMA regardless of whether the ADC
//! is in early interrupt mode or late interrupt mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableAltDMATiming(uint32_t base)
{

    //
    // Enable the Alternate DMA timings wherein DMA is triggered
    // at tDMA.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) | CSL_ADC_ADCCTL1_TDMAEN_MASK));
}

//*****************************************************************************
//
//! Disables alternate timings for DMA trigger
//!
//! \param base is the base address of the ADC module.
//!
//! This function disables the alternate timings(tDMA) for DMA trigger. When
//! disabled the DMA is triggered at the same time as the CPU interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableAltDMATiming(uint32_t base)
{

    //
    // Disable the Alternate DMA timings wherein DMA is triggered at the same
    // time as CPU interrupt.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) & ~CSL_ADC_ADCCTL1_TDMAEN_MASK));
}

//*****************************************************************************
//
//! Enables external channel mux preselection
//!
//! \param base is the base address of the ADC module.
//!
//! This function enables the preselection of external mux pins which is at the
//! end of the S+H window of the previous conversion. This preselection allows
//! some of the external mux settling time to be pipelined with previous
//! conversion's conversion time and can be enabled if ADC SOC sequence is
//! deterministic.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableExtMuxPreselect(uint32_t base)
{

    //
    // Enable the external mux selection at the end of S+H window of
    // previous conversion.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) |
        CSL_ADC_ADCCTL1_EXTMUXPRESELECTEN_MASK));
}

//*****************************************************************************
//
//! Disables external mux preselection
//!
//! \param base is the base address of the ADC module.
//!
//! This function disables the preselection of external mux pins as in
//! ADCEXTMUX pins can only change at the beginning of S+H window of
//! current conversion.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableExtMuxPreselect(uint32_t base)
{

    //
    // Enable the external mux selection at the beginning of S+H window of
    // current conversion.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) &
        ~CSL_ADC_ADCCTL1_EXTMUXPRESELECTEN_MASK));
}

//*****************************************************************************
//
//! Selects the offset trim mode
//!
//! \param base is the base address of the ADC module.
//! \param mode is the offset trim mode to be selected.
//!
//! This function configures the offset trim mode for an ADC instance. This
//! means that the offset trim to be used for an ADC mode would be specified
//! by the \e mode parameter. Valid values for \e mode parameter are:
//! - \e ADC_OFFSET_TRIM_COMMON - Offset trim will always be supplied from
//!   ADCOFFTRIM.OFFTRIM register field (Legacy mode)
//! - \e ADC_OFFSET_TRIM_INDIVIDUAL - offset trim will be supplied from
//!   individual registers based on the combination of resolution, signalmode,
//!   and capacitor bank related configurations.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_selectOffsetTrimMode(uint32_t base, ADC_OffsetTrim mode)
{

    //
    // Enable the external mux selection at the end of S+H window of
    // previous conversion.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL2,
        ((HW_RD_REG16(base + CSL_ADC_ADCCTL2) &
        ~CSL_ADC_ADCCTL2_OFFTRIMMODE_MASK) | (uint16_t)mode));
}

//*****************************************************************************
//
//! Gets the result ready status for ADC interrupt.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the result ready status associated with the selected
//! interrupt number within the ADC wrapper.
//!
//! \e adcIntNum takes a one of the values
//! \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3
//! or \b ADC_INT_NUMBER4 to get the result ready status for the conversions
//! associated with the given interrupt number.
//!
//! \return \b true if the result is available for the selected interrupt
//! and \b false if it is not.
//
//*****************************************************************************
static inline bool
ADC_getIntResultStatus(uint32_t base, ADC_IntNumber adcIntNum)
{

    //
    // Get the specified ADC interrupt result ready status.
    //
    return((HW_RD_REG16(base + CSL_ADC_ADCINTFLG) &
                 (1U << ((uint16_t)adcIntNum + 4U))) != 0U);

}

//*****************************************************************************
//
//! Powers up the analog-to-digital converter core.
//!
//! \param base is the base address of the ADC module.
//!
//! This function powers up the analog circuitry inside the analog core.
//!
//! \note Allow at least a 500us delay before sampling after calling this API.
//! If you enable multiple ADCs, you can delay after they all have begun
//! powering up.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableConverter(uint32_t base)
{
    //
    // Set the bit that powers up the analog circuitry.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) | CSL_ADC_ADCCTL1_ADCPWDNZ_MASK));
}

//*****************************************************************************
//
//! Powers down the analog-to-digital converter module.
//!
//! \param base is the base address of the ADC module.
//!
//! This function powers down the analog circuitry inside the analog core.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableConverter(uint32_t base)
{
    //
    // Clear the bit that powers down the analog circuitry.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL1,
        (HW_RD_REG16(base + CSL_ADC_ADCCTL1) &
        ~CSL_ADC_ADCCTL1_ADCPWDNZ_MASK));
}

//*****************************************************************************
//
//! Forces a SOC flag to a 1 in the analog-to-digital converter.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function forces the SOC flag associated with the SOC specified by
//! \e socNumber. This initiates a conversion once that SOC is given
//! priority. This software trigger can be used whether or not the SOC has been
//! configured to accept some other specific trigger.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceSOC(uint32_t base, ADC_SOCNumber socNumber)
{
    //
    // Write to the register that will force a 1 to the corresponding SOC flag
    //
    HW_WR_REG16(base + CSL_ADC_ADCSOCFRC1, ((uint16_t)1U << (uint16_t)socNumber));
}

//*****************************************************************************
//
//! Forces multiple SOC flags to 1 in the analog-to-digital converter.
//!
//! \param base is the base address of the ADC module.
//! \param socMask is the SOCs to be forced through software
//!
//! This function forces the SOCFRC1 flags associated with the SOCs specified
//! by \e socMask. This initiates a conversion once the desired SOCs are given
//! priority. This software trigger can be used whether or not the SOC has been
//! configured to accept some other specific trigger.
//! Valid values for \e socMask parameter can be any of the individual
//! ADC_FORCE_SOCx values or any of their OR'd combination to trigger multiple
//! SOCs.
//!
//! \note To trigger SOC0, SOC1 and SOC2, value (ADC_FORCE_SOC0 |
//! ADC_FORCE_SOC1 | ADC_FORCE_SOC2) should be passed as socMask.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceMultipleSOC(uint32_t base, uint16_t socMask)
{
    //
    // Write to the register that will force a 1 to desired SOCs
    //
    HW_WR_REG16(base + CSL_ADC_ADCSOCFRC1, socMask);
}

//*****************************************************************************
//
//! Gets the current ADC interrupt status.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the interrupt status for the analog-to-digital
//! converter.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be cleared.
//!
//! \return \b true if the interrupt flag for the specified interrupt number is
//! set and \b false if it is not.
//
//*****************************************************************************
static inline bool
ADC_getInterruptStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Get the specified ADC interrupt status.
    //
    return((HW_RD_REG16(base + CSL_ADC_ADCINTFLG) &
        (1U << (uint16_t)adcIntNum)) != 0U);
}

//*****************************************************************************
//
//! Clears ADC interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function clears the specified ADC interrupt sources so that they no
//! longer assert. If not in continuous mode, this function must be called
//! before any further interrupt pulses may occur.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be cleared.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearInterruptStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Clear the specified interrupt.
    //
    HW_WR_REG16(base + CSL_ADC_ADCINTFLGCLR, ((uint16_t)1U << (uint16_t)adcIntNum));
}

//*****************************************************************************
//
//! Gets the current ADC interrupt overflow status.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the interrupt overflow status for the
//! analog-to-digital converter. An overflow condition is generated
//! irrespective of the continuous mode.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be cleared.
//!
//! \return \b true if the interrupt overflow flag for the specified interrupt
//! number is set and \b false if it is not.
//
//*****************************************************************************
static inline bool
ADC_getInterruptOverflowStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Get the specified ADC interrupt status.
    //
    return((HW_RD_REG16(base + CSL_ADC_ADCINTOVF) &
        (1U << (uint16_t)adcIntNum)) != 0U);
}

//*****************************************************************************
//
//! Clears ADC interrupt overflow sources.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function clears the specified ADC interrupt overflow sources so that
//! they no longer assert. If software tries to clear the overflow in the same
//! cycle that hardware tries to set the overflow, then hardware has priority.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be cleared.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearInterruptOverflowStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Clear the specified interrupt overflow bit.
    //
    HW_WR_REG16(base + CSL_ADC_ADCINTOVFCLR, ((uint16_t)1U << (uint16_t)adcIntNum));
}

//*****************************************************************************
//
//! Reads the conversion result.
//!
//! \param resultBase is the base address of the ADC results.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function returns the conversion result that corresponds to the base
//! address passed into \e resultBase and the SOC passed into \e socNumber.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC's result is to be read.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the conversion result.
//
//*****************************************************************************
static inline uint16_t
ADC_readResult(uint32_t resultBase, ADC_SOCNumber socNumber)
{
    //
    // Return the ADC result for the selected SOC.
    //
    return(HW_RD_REG16(resultBase + CSL_ADC_RESULT_ADCRESULT0 +
        ((uint32_t)socNumber * ADC_RESULT_ADCRESULTx_STEP)));
}

//*****************************************************************************
//
//! Determines whether the ADC is busy or not.
//!
//! \param base is the base address of the ADC.
//!
//! This function allows the caller to determine whether or not the ADC is
//! busy and can sample another channel.
//!
//! \return Returns \b true if the ADC is sampling or \b false if all
//! samples are complete.
//
//*****************************************************************************
static inline bool
ADC_isBusy(uint32_t base)
{
    //
    // Determine if the ADC is busy.
    //
    return((HW_RD_REG16(base + CSL_ADC_ADCCTL1) &
        CSL_ADC_ADCCTL1_ADCBSY_MASK) != 0U);
}

//*****************************************************************************
//
//! Set SOC burst mode.
//!
//! \param base is the base address of the ADC.
//! \param trigger the source that will cause the burst conversion sequence.
//! \param burstSize is the number of SOCs converted during a burst sequence.
//!
//! This function configures the burst trigger and burstSize of an ADC module.
//! Burst mode allows a single trigger to walk through the round-robin SOCs one
//! or more at a time. When burst mode is enabled, the trigger selected by the
//! ADC_setupSOC() API will no longer have an effect on the SOCs in round-robin
//! mode. Instead, the source specified through the \e trigger parameter will
//! cause a burst of \e burstSize conversions to occur.
//!
//! The \e trigger parameter takes the same values as the ADC_setupSOC() API
//! The \e burstSize parameter should be a value between 1 and 16 inclusive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setBurstModeConfig(uint32_t base, ADC_Trigger trigger, uint16_t burstSize)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    DebugP_assert(((uint32_t)trigger & ~0x7FU) == 0U);
    DebugP_assert((burstSize >= 1U) && (burstSize <= 16U));

    //
    // Write the burst mode configuration to the register.
    //
    regValue = (uint16_t)trigger |
        ((burstSize - 1U) << CSL_ADC_ADCBURSTCTL_BURSTSIZE_SHIFT);

    HW_WR_REG16(base + CSL_ADC_ADCBURSTCTL,
        ((HW_RD_REG16(base + CSL_ADC_ADCBURSTCTL) &
        ~((uint16_t)CSL_ADC_ADCBURSTCTL_BURSTTRIGSEL_MASK |
        CSL_ADC_ADCBURSTCTL_BURSTSIZE_MASK)) | regValue));
}

//*****************************************************************************
//
//! Enables SOC burst mode.
//!
//! \param base is the base address of the ADC.
//!
//! This function enables SOC burst mode operation of the ADC. Burst mode
//! allows a single trigger to walk through the round-robin SOCs one or more at
//! a time. When burst mode is enabled, the trigger selected by the
//! ADC_setupSOC() API will no longer have an effect on the SOCs in round-robin
//! mode. Use ADC_setBurstMode() to configure the burst trigger and size.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableBurstMode(uint32_t base)
{
    //
    // Enable burst mode.
    //
    HW_WR_REG16(base + CSL_ADC_ADCBURSTCTL,
        (HW_RD_REG16(base + CSL_ADC_ADCBURSTCTL) |
        CSL_ADC_ADCBURSTCTL_BURSTEN_MASK));
}

//*****************************************************************************
//
//! Disables SOC burst mode.
//!
//! \param base is the base address of the ADC.
//!
//! This function disables SOC burst mode operation of the ADC. SOCs in
//! round-robin mode will be triggered by the trigger configured using the
//! ADC_setupSOC() API.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableBurstMode(uint32_t base)
{
    //
    // Disable burst mode.
    //
    HW_WR_REG16(base + CSL_ADC_ADCBURSTCTL,
        (HW_RD_REG16(base + CSL_ADC_ADCBURSTCTL) &
        ~CSL_ADC_ADCBURSTCTL_BURSTEN_MASK));
}

//*****************************************************************************
//
//! Sets the priority mode of the SOCs.
//!
//! \param base is the base address of the ADC.
//! \param priMode is the priority mode of the SOCs.
//!
//! This function sets the priority mode of the SOCs. There are three main
//! modes that can be passed in the \e priMode parameter
//!
//! - All SOCs are in round-robin mode. This means no SOC has an inherent
//! higher priority over another. This is selected by passing in the value
//! \b ADC_PRI_ALL_ROUND_ROBIN.
//! - All priorities are in high priority mode. This means that the priority of
//! the SOC is determined by its SOC number. This option is selected by passing
//! in the value \b ADC_PRI_ALL_HIPRI.
//! - A range of SOCs are assigned high priority, with all others in round
//! robin mode. High priority mode means that an SOC with high priority will
//! interrupt the round robin wheel and insert itself as the next conversion.
//! Passing in the value \b ADC_PRI_SOC0_HIPRI will make SOC0 highest priority,
//! \b ADC_PRI_THRU_SOC1_HIPRI will put SOC0 and SOC 1 in high priority, and so
//! on up to \b ADC_PRI_THRU_SOC14_HIPRI where SOCs 0 through 14 are in high
//! priority.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setSOCPriority(uint32_t base, ADC_PriorityMode priMode)
{
    //
    // Set SOC priority
    //
    HW_WR_REG16(base + CSL_ADC_ADCSOCPRICTL,
        ((HW_RD_REG16(base + CSL_ADC_ADCSOCPRICTL) &
        ~CSL_ADC_ADCSOCPRICTL_SOCPRIORITY_MASK) | (uint16_t)priMode));
}

//*****************************************************************************
//
//! Configures Open/Shorts Detection Circuit Mode.
//!
//! \param base is the base address of the ADC.
//! \param modeVal is the desired open/shorts detection circuit mode.
//!
//! This function configures the open/shorts detection circuit mode of the ADC.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_configOSDetectMode(uint32_t base, ADC_OSDetectMode modeVal)
{
    //
    // Configure open/shorts detection circuit mode.
    //
    HW_WR_REG16(base + CSL_ADC_ADCOSDETECT,
        ((HW_RD_REG16(base + CSL_ADC_ADCOSDETECT) &
        ~CSL_ADC_ADCOSDETECT_DETECTCFG_MASK) | (uint16_t)modeVal));
}

//*****************************************************************************
//
//! Configures a post-processing block (PPB) in the ADC.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function associates a post-processing block with a SOC.
//!
//! The \e ppbNumber is a value \b ADC_PPB_NUMBERX where X is a value from 1 to
//! 4 inclusive that identifies a PPB to be configured.  The \e socNumber
//! number is a value \b ADC_SOC_NUMBERX where X is a number from 0 to 15
//! specifying which SOC is to be configured on the ADC module specified by
//! \e base.
//!
//! \note You can have more that one PPB associated with the same SOC, but a
//! PPB can only be configured to correspond to one SOC at a time. Also note
//! that when you have multiple PPBs for the same SOC, the calibration offset
//! that actually gets applied will be that of the PPB with the highest number.
//! Since SOC0 is the default for all PPBs, look out for unintentional
//! overwriting of a lower numbered PPB's offset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setupPPB(uint32_t base, ADC_PPBNumber ppbNumber, ADC_SOCNumber socNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1CONFIG;

    //
    // Write the configuration to the register.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG_CONFIG_MASK) |
        ((uint16_t)socNumber & CSL_ADC_ADCPPB1CONFIG_CONFIG_MASK)));
}

//*****************************************************************************
//
//! Enables individual ADC PPB event sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event sources to be enabled.
//!
//! This function enables the indicated ADC PPB event sources.  This will allow
//! the specified events to propagate through the X-BAR to a pin or to an ePWM
//! module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEvent(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    DebugP_assert((evtFlags & ~0x7U) == 0U);

    //
    // Enable the specified event.
    //
    HW_WR_REG16(base + CSL_ADC_ADCEVTSEL,
        (HW_RD_REG16(base + CSL_ADC_ADCEVTSEL) |
        (evtFlags << ((uint16_t)ppbNumber * 4U))));
}

//*****************************************************************************
//
//! Disables individual ADC PPB event sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event sources to be enabled.
//!
//! This function disables the indicated ADC PPB event sources.  This will stop
//! the specified events from propagating through the X-BAR to other modules.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEvent(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    DebugP_assert((evtFlags & ~0x7U) == 0U);

    //
    // Disable the specified event.
    //
    HW_WR_REG16(base + CSL_ADC_ADCEVTSEL,
        (HW_RD_REG16(base + CSL_ADC_ADCEVTSEL) &
        ~(evtFlags << ((uint16_t)ppbNumber * 4U))));
}

//*****************************************************************************
//
//! Enables individual ADC PPB event interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param intFlags is a bit mask of the interrupt sources to be enabled.
//!
//! This function enables the indicated ADC PPB interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEventInterrupt(uint32_t base, ADC_PPBNumber ppbNumber,
                            uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    DebugP_assert((intFlags & ~0x7U) == 0U);

    //
    // Enable the specified event interrupts.
    //
    HW_WR_REG16(base + CSL_ADC_ADCEVTINTSEL,
        (HW_RD_REG16(base + CSL_ADC_ADCEVTINTSEL) |
        (intFlags << ((uint16_t)ppbNumber * 4U))));
}

//*****************************************************************************
//
//! Disables individual ADC PPB event interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param intFlags is a bit mask of the interrupt source to be disabled.
//!
//! This function disables the indicated ADC PPB interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEventInterrupt(uint32_t base, ADC_PPBNumber ppbNumber,
                             uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    DebugP_assert((intFlags & ~0x7U) == 0U);

    //
    // Disable the specified event interrupts.
    //
    HW_WR_REG16(base + CSL_ADC_ADCEVTINTSEL,
        (HW_RD_REG16(base + CSL_ADC_ADCEVTINTSEL) &
        ~(intFlags << ((uint16_t)ppbNumber * 4U))));
}

//*****************************************************************************
//
//! Gets the current ADC event status.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the event status for the analog-to-digital converter.
//!
//! \return Returns the current event status
//
//*****************************************************************************
static inline uint16_t
ADC_getPPBEventStatus(uint32_t base, ADC_PPBNumber ppbNumber)
{
    //
    // Get the event status for the specified post-processing block.
    //
    return((HW_RD_REG16(base + CSL_ADC_ADCEVTSTAT) >>
        ((uint16_t)ppbNumber * 4U)) & 0x7U);
}

//*****************************************************************************
//
//! Clears ADC event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event source to be cleared.
//!
//! This function clears the indicated ADC PPB event flags. After an event
//! occurs this function must be called to allow additional events to be
//! produced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearPPBEventStatus(uint32_t base, ADC_PPBNumber ppbNumber,
                        uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    DebugP_assert((evtFlags & ~0x7U) == 0U);

    //
    // Clear the specified event interrupts.
    //
    HW_WR_REG16(base + CSL_ADC_ADCEVTCLR,
        (HW_RD_REG16(base + CSL_ADC_ADCEVTCLR) |
        (evtFlags << ((uint16_t)ppbNumber * 4U))));
}

//*****************************************************************************
//
//! Enables cycle-by-cycle clear of ADC PPB event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables the automatic cycle-by-cycle clear of ADC PPB event
//! flags. When enabled, the desired PPB event flags are automatically cleared
//! on the next ADCPPBxRESULT load, unless a set condition is also occurring at
//! the same time, in which case the set takes precedence.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEventCBCClear(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG;

    //
    // Set automatic cycle-by-cycle flag clear bit
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) | CSL_ADC_ADCPPB1CONFIG_CBCEN_MASK));
}

//*****************************************************************************
//
//! Disables cycle-by-cycle clear of ADC PPB event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables the cycle-by-cycle clear of ADC PPB event flags. When
//! disabled, the desired PPB event flags are to be cleared explicitly in
//! software inorder to generate next set of interrupts/events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEventCBCClear(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG;

    //
    // Clear automatic cycle-by-cycle flag clear bit
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG_CBCEN_MASK));
}

//*****************************************************************************
//
//! Configures PPB count limit.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param limit is the desired PPB count limit.
//!
//! This function configures the PPB oversampling count limit which defines the
//! number of ADC conversions to accumulate before partial sum is automatically
//! loaded to the sum registes.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setPPBCountLimit(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t limit)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    DebugP_assert(limit <= CSL_ADC_ADCPPB1LIMIT_LIMIT_MAX);

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxLIMIT_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1LIMIT;

    //
    // Enable PPB two's complement.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1LIMIT_LIMIT_MASK) |
        (limit << CSL_ADC_ADCPPB1LIMIT_LIMIT_SHIFT)));

}

//*****************************************************************************
//
//! Returns the PPB count limit.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the PPB oversampling count limit which defines the
//! number of ADC conversions to accumulate before partial sum is automatically
//! loaded to the sum registes.
//!
//! \return Returns the PPB count limit.
//
//*****************************************************************************
static inline uint16_t
ADC_getPPBCountLimit(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint16_t limit;
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxLIMIT_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1LIMIT;

    limit = (HW_RD_REG16(base + ppbOffset) &
        ~(CSL_ADC_ADCPPB1LIMIT_LIMIT_MASK)) >> CSL_ADC_ADCPPB1LIMIT_LIMIT_SHIFT;
    return(limit);
}

//*****************************************************************************
//
//! Reads the oversampled partial count from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled partial count that corresponds to
//! the base address passed into \e base and the PPB passed into \e ppbNumber.
//!
//! \return Returns the oversampled partial count value.
//
//*****************************************************************************
static inline uint16_t
ADC_readPPBPCount(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the partial count of the selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPBP1PCOUNT +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPCOUNT_STEP)));
}

//*****************************************************************************
//
//! Reads the oversampled partial sum from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled partial sum of PPB results that
//! corresponds to the base address passed into \e base and the PPB passed
//! into \e ppbNumber.
//!
//! \return Returns the oversampled partial sum value.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBPSum(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the partial sum result of selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPB1PSUM +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPSUM_STEP)));
}

//*****************************************************************************
//
//! Reads the processed conversion result's partial maximum value from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled partial maximum that corresponds to
//! the base address passed into \e base and the PPB passed into \e ppbNumber.
//!
//! \return Returns the oversampled partial maximum value.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBPMax(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Return the partial maximum value of selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPB1PMAX +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPMAX_STEP)));
}

//*****************************************************************************
//
//! Reads the processed conversion result's partial minimum value from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled partial minimum that corresponds to
//! the base address passed into \e base and the PPB passed into \e ppbNumber.
//!
//! \return Returns the oversampled partial minimum value.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBPMin(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Return the partial minimum value of selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPB1PMIN +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPMIN_STEP)));
}

//*****************************************************************************
//
//! Reads the index of the result with partial maximum value from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the index of the oversampled partial maximum value
//! that corresponds to the base address passed into \e base
//! and the PPB passed into \e ppbNumber.
//!
//! \return Returns the index of the oversampled partial maximum value.
//
//*****************************************************************************
static inline uint16_t
ADC_readPPBPMaxIndex(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the index of the partial maximum value of selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPB1PMAXI +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPMAXI_STEP)));
}

//*****************************************************************************
//
//! Reads the index of the result with partial minimum value from the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the index of the oversampled partial minimum value
//! that corresponds to the base address passed into \e base
//! and the PPB passed into \e ppbNumber.
//!
//! \return Returns the index of the oversampled partial minimum value.
//
//*****************************************************************************
static inline uint16_t
ADC_readPPBPMinIndex(uint32_t base, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the index of the partial minimum value of selected PPB.
    //
    return(HW_RD_REG32(base + (uint32_t)CSL_ADC_ADCPPB1PMINI +
            ((uint32_t)ppbNumber * ADC_ADCPPBxPMINI_STEP)));
}

//*****************************************************************************
//
//! Enables absolute value capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables the absolute value functionality in the post-
//! processing block specified by the \e ppbNumber parameter. When enabled,
//! absolute value calculation would be done on the ADC Result associated
//! with the selected SOC. In other words, the PPB result will calculated as
//! shown below: (ADCPPBxRESULT = abs(ADCRESULTx - ADCPPBxOFFREF))
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_enablePPBAbsoluteValue(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG;

    //
    // Enable PPB absolute value.
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) | CSL_ADC_ADCPPB1CONFIG_ABSEN_MASK));
}

//*****************************************************************************
//
//! Disables absolute value capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables the absolute value functionality in the post-
//! processing block specified by the \e ppbNumber parameter. When disabled,
//! absolute value calculation would be done on the ADC Result associated
//! with the selected SOC. In other words, the PPB result will calculated as
//! shown below: (ADCPPBxRESULT = ADCRESULTx - ADCPPBxOFFREF)
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_disablePPBAbsoluteValue(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG;

    //
    // Disable PPB abosulte value.
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG_ABSEN_MASK));
}

//*****************************************************************************
//
//! Configures PPB shift value.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param shiftVal is the number of bits to right shift PSUM before loading
//! to final PPB SUM.
//!
//! This function configured the shift value required to right shift the PPB
//! PSUM before loading into the final PPB SUM.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_setPPBShiftValue(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t shiftVal)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    DebugP_assert(shiftVal <= 10U);

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

    //
    // Configure shift value for the PPB.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG2_SHIFT_MASK) |
        (shiftVal << CSL_ADC_ADCPPB1CONFIG2_SHIFT_SHIFT)));

}

//*****************************************************************************
//
//! Configures PPB sync input.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param syncInput is the desired sync event to transfer partial registers
//! to final registers and reset the partial registers.
//!
//! This function configures desired sync event to transfer partial registers
//! to final registers and reset the partial registers. For valid values of
//! \e syncInput refer to enum \e ADC_SyncInput.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_selectPPBSyncInput(uint32_t base, ADC_PPBNumber ppbNumber,
                       uint16_t syncInput)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

    //
    // Select sync input for the PPB.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG2_SYNCINSEL_MASK) |
        (syncInput << CSL_ADC_ADCPPB1CONFIG2_SYNCINSEL_SHIFT)));

}

//*****************************************************************************
//
//! Forces PPB software sync.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function forces the software sync for the desired PPB.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_forcePPBSync(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

    //
    // Force software sync for the PPB.
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) | CSL_ADC_ADCPPB1CONFIG2_SWSYNC_MASK));

}

//*****************************************************************************
//
//! Configures PPB interrupt source.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param osIntSrc selects PPB interrupt source.
//!
//! This function sets which oversampling event is the source of
//! a PPB intterupt.
//!
//! For valid values of \e osIntSrc refer to enum \e ADC_PPBIntSrcSelect.
//!
//! \return None
//*****************************************************************************
static inline void
ADC_selectPPBOSINTSource(uint32_t base, ADC_PPBNumber ppbNumber,
                         uint16_t osIntSrc)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    DebugP_assert(osIntSrc <= 1U);

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

    //
    // Select PPB OSINT source.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) &
        ~CSL_ADC_ADCPPB1CONFIG2_OSINTSEL_MASK) |
        (osIntSrc << CSL_ADC_ADCPPB1CONFIG2_OSINTSEL_SHIFT)));

}

//*****************************************************************************
//
//! Configures PPB compare source.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param compSrc is the desired source to be used for zero crossing detect
//! logic and threshold compare.
//!
//! This function configures the desired source to be used for zero crossing
//! detect logic and threshold compare. For valid values of \e compSrc refer to
//! enum \e ADC_PPBCompSource.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_selectPPBCompareSource(uint32_t base, ADC_PPBNumber ppbNumber,
                           uint16_t compSrc)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    DebugP_assert(compSrc <= 2U);

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

    //
    // Select PPB compare source..
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1CONFIG2_COMPSEL_MASK) |
        (compSrc << CSL_ADC_ADCPPB1CONFIG2_COMPSEL_SHIFT)));

}

//*****************************************************************************
//
//! Reads the oversampled final sum from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the processed sum of results that corresponds to
//! the base address passed into \e resultBase and the PPB passed into
//! \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the oversampled final sum value.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBSum(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Return the result of selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1SUM +
            ((uint32_t)ppbNumber * 8UL)));

}

//*****************************************************************************
//
//! Reads the oversampled final count from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled final count that
//! corresponds to the base address passed into \e resultBase and the
//! PPB passed into \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the oversampled final count value.
//
//*****************************************************************************
static inline uint32_t
ADC_readPPBCount(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Return the final count of selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1COUNT +
            ((uint32_t)ppbNumber * 8UL)));

}

//*****************************************************************************
//
//! Reads the processed conversion result's maximum value from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the oversampled final maximum that corresponds to
//! the base address passed into \e resultBase and the PPB passed into
//! \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the oversampled final maximum value.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBMax(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Return the final maximum value of selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1MAX +
            ((uint32_t)ppbNumber * 16UL)));

}

//*****************************************************************************
//
//! Reads the processed conversion result's minimum value from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the processed conversion result that corresponds to
//! the base address passed into \e resultBase and the PPB passed into
//! \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the signed 32-bit conversion result.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBMin(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Return the final minimum value of selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1MIN +
            ((uint32_t)ppbNumber * 16UL)));

}

//*****************************************************************************
//
//! Reads the index of the result with maximum value from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the index of the processed conversion's maximum value
//! that corresponds to the base address passed into \e resultBase and
//! the PPB passed into \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the index of the result with maximum value.
//
//*****************************************************************************
static inline uint16_t
ADC_readPPBMaxIndex(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the index of the final maximum value of selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1MAXI +
            ((uint32_t)ppbNumber * 16UL)));

}

//*****************************************************************************
//
//! Reads the index of the result with minimum value from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the index of the processed conversion's minimum value
//! that corresponds to the base address passed into \e resultBase and
//! the PPB passed into \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the index of the result with final minimum value.
//
//*****************************************************************************
static inline uint16_t
ADC_readPPBMinIndex(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{

    //
    // Returns the index of the final minimum value of the selected PPB.
    //
    return(HW_RD_REG32(resultBase + (uint32_t)CSL_ADC_RESULT_ADCPPB1MINI +
            ((uint32_t)ppbNumber * 16UL)));

}

//*****************************************************************************
//
//! Reads the processed conversion result from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the processed conversion result that corresponds to
//! the base address passed into \e resultBase and the PPB passed into
//! \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the signed 32-bit conversion result.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBResult(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{
    //
    // Return the result of selected PPB.
    //
    return((int32_t)HW_RD_REG32(resultBase + CSL_ADC_RESULT_ADCPPB1RESULT +
           ((uint32_t)ppbNumber * ADC_RESULT_ADCPPBxRESULT_STEP)));
}

//*****************************************************************************
//
//! Reads sample delay time stamp from a PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the sample delay time stamp. This delay is the number
//! of system clock cycles between the SOC being triggered and when it began
//! converting.
//!
//! \return Returns the delay time stamp.
//
//*****************************************************************************
static inline uint16_t
ADC_getPPBDelayTimeStamp(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate delay.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1STAMP;

    //
    // Return the delay time stamp.
    //
    return(HW_RD_REG16(base + ppbOffset) & CSL_ADC_ADCPPB1STAMP_DLYSTAMP_MASK);
}

//*****************************************************************************
//
//! Sets the post processing block offset correction.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param offset is the 10-bit signed value subtracted from ADC the output.
//!
//! This function sets the PPB offset correction value.  This value can be used
//! to digitally remove any system-level offset inherent in the ADCIN circuit
//! before it is stored in the appropriate result register. The \e offset
//! parameter is \b subtracted from the ADC output and is a signed value from
//! -512 to 511 inclusive. For example, when \e offset = 1, ADCRESULT = ADC
//! output - 1. When \e offset = -512, ADCRESULT = ADC output - (-512) or ADC
//! output + 512.
//!
//! Passing a zero in to the \e offset parameter will effectively disable the
//! calculation, allowing the raw ADC result to be passed unchanged into the
//! result register.
//!
//! \note If multiple PPBs are applied to the same SOC, the offset that will be
//! applied will be that of the PPB with the highest number.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_setPPBCalibrationOffset(uint32_t base, ADC_PPBNumber ppbNumber,
                            int16_t offset)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate offset register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1OFFCAL;

    //
    // Write the offset amount.
    //
    HW_WR_REG16(base + ppbOffset,
        ((HW_RD_REG16(base + ppbOffset) & ~CSL_ADC_ADCPPB1OFFCAL_OFFCAL_MASK) |
        ((uint16_t)offset & CSL_ADC_ADCPPB1OFFCAL_OFFCAL_MASK)));
}

//*****************************************************************************
//
//! Sets the post processing block reference offset.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param offset is the 12-bit unsigned value subtracted from ADC the output.
//!
//! This function sets the PPB reference offset value. This can be used to
//! either calculate the feedback error or convert a unipolar signal to bipolar
//! by subtracting a reference value. The result will be stored in the
//! appropriate PPB result register which can be read using ADC_readPPBResult().
//!
//! Passing a zero in to the \e offset parameter will effectively disable the
//! calculation and will pass the ADC result to the PPB result register
//! unchanged.
//!
//! \note If in 12-bit mode, you may only pass a 12-bit value into the \e offset
//! parameter.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_setPPBReferenceOffset(uint32_t base, ADC_PPBNumber ppbNumber,
                          uint16_t offset)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate offset register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1OFFREF;

    //
    // Write the offset amount.
    //
    HW_WR_REG16(base + ppbOffset, offset);
}

//*****************************************************************************
//
//! Enables two's complement capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables two's complement in the post-processing block
//! specified by the \e ppbNumber parameter. When enabled, a two's complement
//! will be performed on the output of the offset subtraction before it is
//! stored in the appropriate PPB result register. In other words, the PPB
//! result will be the reference offset value minus the the ADC result value
//! (ADCPPBxRESULT = ADCSOCxOFFREF - ADCRESULTx).
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_enablePPBTwosComplement(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1CONFIG;

    //
    // Enable the twos complement
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) |
        CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_MASK));
}

//*****************************************************************************
//
//! Disables two's complement capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables two's complement in the post-processing block
//! specified by the \e ppbNumber parameter. When disabled, a two's complement
//! will \b NOT be performed on the output of the offset subtraction before it
//! is stored in the appropriate PPB result register. In other words, the PPB
//! result will be the ADC result value minus the reference offset value
//! (ADCPPBxRESULT = ADCRESULTx - ADCSOCxOFFREF).
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_disablePPBTwosComplement(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
        CSL_ADC_ADCPPB1CONFIG;

    //
    // Disable the twos complement
    //
    HW_WR_REG16(base + ppbOffset,
        (HW_RD_REG16(base + ppbOffset) &
        ~CSL_ADC_ADCPPB1CONFIG_TWOSCOMPEN_MASK));
}

//*****************************************************************************
//
//! Enables the extended low limit in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables the low limit for PPB by allowing the PPB low
//! limit to be set by ADCPPBxTRIPLO2 register.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_enablePPBExtendedLowLimit(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbLoOffset;

    //
    // Get the offset to the appropriate trip limit registers.
    //
    ppbLoOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  CSL_ADC_ADCPPB1TRIPLO;

    //
    // Enable PPB extended low limit.
    //
    HW_WR_REG32(base + ppbLoOffset,
       (HW_RD_REG32(base + ppbLoOffset) | CSL_ADC_ADCPPB1TRIPLO_LIMITLO2EN_MASK));

}

//*****************************************************************************
//
//! Disables extended low limit capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables the low limit for PPB by allowing the PPB low
//! limit to be set by ADCPPBxTRIPLO register.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_disablePPBExtendedLowLimit(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  CSL_ADC_ADCPPB1TRIPLO;

    //
    // Disable PPB extended low limit.
    //
    HW_WR_REG32(base + ppbOffset,
        (HW_RD_REG32(base + ppbOffset) &
        ~CSL_ADC_ADCPPB1TRIPLO_LIMITLO2EN_MASK));

}

//*****************************************************************************
//
//! Enables an ADC interrupt source.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function enables the indicated ADC interrupt source.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2,
//! \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express which of the four
//! interrupts of the ADC module should be enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableInterrupt(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + CSL_ADC_ADCINTSEL1N2 +
        (((uint32_t)adcIntNum >> 1) * ADC_ADCINTSELxNy_STEP);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Enable the specified ADC interrupt.
    //
    HW_WR_REG16(intRegAddr,
        HW_RD_REG16(intRegAddr) |
        (CSL_ADC_ADCINTSEL1N2_INT1E_MASK << shiftVal));
}

//*****************************************************************************
//
//! Disables an ADC interrupt source.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function disables the indicated ADC interrupt source.
//! Only the sources that are enabled can be reflected to the processor
//! interrupt. Disabled sources have no effect on the processor.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2,
//! \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express which of the four
//! interrupts of the ADC module should be disabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableInterrupt(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + CSL_ADC_ADCINTSEL1N2 +
        (((uint32_t)adcIntNum >> 1) * ADC_ADCINTSELxNy_STEP);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Disable the specified ADC interrupt.
    //
    HW_WR_REG16(intRegAddr,
        HW_RD_REG16(intRegAddr) &
        ~(CSL_ADC_ADCINTSEL1N2_INT1E_MASK << shiftVal));
}

//*****************************************************************************
//
//! Sets the source EOC for an analog-to-digital converter interrupt.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//! \param intTrigger is the interrupt source EOCx, OSINTy are used.
//!
//! This function sets which conversion is the source of an ADC interrupt.
//!
//! The \e intTrigger number is a value \b ADC_INT_TRIGGER_X where X specifies
//! the interrupt trigger to be configured for the ADC module specified by
//! \e base. Refer \b ADC_IntTrigger enum for valid values for this input.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2,
//! \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express which of the four
//! interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptSource(uint32_t base, ADC_IntNumber adcIntNum,
                       uint16_t intTrigger)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + CSL_ADC_ADCINTSEL1N2 +
        (((uint32_t)adcIntNum >> 1) * ADC_ADCINTSELxNy_STEP);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Set the specified ADC interrupt source.
    //
    HW_WR_REG16(intRegAddr,
        ((HW_RD_REG16(intRegAddr) &
        ~(CSL_ADC_ADCINTSEL1N2_INT1SEL_MASK << shiftVal)) |
        ((uint16_t)intTrigger << shiftVal)));
}

//*****************************************************************************
//
//! Enables continuous mode for an ADC interrupt.
//!
//! \param base is the base address of the ADC.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function enables continuous mode for the ADC interrupt passed into
//! \e adcIntNum. This means that pulses will be generated for the specified
//! ADC interrupt whenever an EOC pulse is generated irrespective of whether or
//! not the flag bit is set.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2,
//! \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express which of the four
//! interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableContinuousMode(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + CSL_ADC_ADCINTSEL1N2 +
        (((uint32_t)adcIntNum >> 1) * ADC_ADCINTSELxNy_STEP);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Enable continuous mode for the specified ADC interrupt.
    //
    HW_WR_REG16(intRegAddr,
        HW_RD_REG16(intRegAddr) |
        (CSL_ADC_ADCINTSEL1N2_INT1CONT_MASK << shiftVal));
}

//*****************************************************************************
//
//! Disables continuous mode for an ADC interrupt.
//!
//! \param base is the base address of the ADC.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function disables continuous mode for the ADC interrupt passed into
//! \e adcIntNum. This means that pulses will not be generated for the
//! specified ADC interrupt until the corresponding interrupt flag for the
//! previous interrupt occurrence has been cleared using
//! ADC_clearInterruptStatus().
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1, \b ADC_INT_NUMBER2,
//! \b ADC_INT_NUMBER3, or \b ADC_INT_NUMBER4 to express which of the four
//! interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableContinuousMode(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + CSL_ADC_ADCINTSEL1N2 +
        (((uint32_t)adcIntNum >> 1) * ADC_ADCINTSELxNy_STEP);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Disable continuous mode for the specified ADC interrupt.
    //
    HW_WR_REG16(intRegAddr,
        HW_RD_REG16(intRegAddr) &
        ~(CSL_ADC_ADCINTSEL1N2_INT1CONT_MASK << shiftVal));
}

//*****************************************************************************
//
//! Configures the safety checker result for a selected SOC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param scInput is the desired input configuration.
//!
//! This function configures the safety checker input for the desired SOC.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e scInput is the desired safety checker input configuration.
//! Valid values can be refered from the enum \e ADC_SafetyCheckerInput.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_configSOCSafetyCheckerInput(uint32_t base, ADC_SOCNumber socNumber,
                                ADC_SafetyCheckerInput scInput)
{
    uint32_t socShift;

    //
    // Calculate the SOC shift.
    //
    socShift = ((uint32_t)socNumber * 2U);

    //
    // Configure the Safety Checker Result mode.
    //
    HW_WR_REG32(base + CSL_ADC_ADCSAFECHECKRESEN,
        ((HW_RD_REG32(base + CSL_ADC_ADCSAFECHECKRESEN) &
        ~(CSL_ADC_ADCSAFECHECKRESEN_SOC0CHKEN_MASK << socShift)) |
        ((uint32_t)scInput << socShift)));
}

//*****************************************************************************
//
//! Enables the ADC result safety checker module.
//!
//! \param scBase is the base address of the ADC Safety Checker module.
//!
//! This function enables the ADC result safety checker module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableSafetyChecker(uint32_t scBase)
{

    //
    // Enable the Saftey Checker module
    //
    HW_WR_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG,
        (HW_RD_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG) |
        CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK));
}

//*****************************************************************************
//
//! Disables the ADC result safety checker module.
//!
//! \param scBase is the base address of the ADC Safety Checker module.
//!
//! This function disables the ADC result safety checker module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableSafetyChecker(uint32_t scBase)
{

    //
    // Disable the Saftey Checker module.
    //
    HW_WR_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG,
        (HW_RD_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG) &
        ~CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK));
}

//*****************************************************************************
//
//! Forces the software sync for the safety checker module
//!
//! \param scBase is the base address of the safety checker module.
//!
//! This function forces the software sync for the safety checker module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceSafetyCheckerSync(uint32_t scBase)
{

    //
    // Force software sync for the safety checker module.
    //
    HW_WR_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG,
        (HW_RD_REG16(scBase + CSL_ADC_SAFETY_CHECKCONFIG) |
        CSL_ADC_SAFETY_CHECKCONFIG_SWSYNC_MASK));
}

//*****************************************************************************
//
//! Returns the status of the safey checker module.
//!
//! \param scBase is the base address of the ADC safety checker module.
//!
//! This function configures the external channel for an SOC by configuring
//! the external channel mux for an SOC.
//!
//! \return Returns the status of the safety checker module. Valid values
//! are any of the follwing values or any of their OR'd combination:
//! - \e ADC_SAFECHECK_RESULT1_READY
//! - \e ADC_SAFECHECK_RESULT2_READY
//! - \e ADC_SAFECHECK_RESULT1_OOT
//
//*****************************************************************************
static inline uint16_t
ADC_getSafetyCheckerStatus(uint32_t scBase)
{

    //
    // Returns Safety Checker module status
    //
    return(HW_RD_REG16(scBase + CSL_ADC_SAFETY_CHECKSTATUS) &
        ADC_SAFECHECK_STATUS_MASK);
}

//*****************************************************************************
//
//! Configures the safety checker module.
//!
//! \param scBase is the base address of the ADC safety checker module.
//! \param checkInst is the safety checker instance to be configured.
//! \param adcInst is the desired ADC instance.
//! \param adcResultInst is the desired ADC result instance.
//!
//! This function configures the selected checker instance of a safety checker
//! instance.
//!
//! The \e checkInst parameter can take values defined by enum
//! \b ADC_SafetyCheckInst. The \e adcInst parameter can take values defined
//! by enum \b ADC_Select. The \e adcResultInst parameter can take values
//! defined by enum \b ADC_ResultSelect.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_configureSafetyChecker(uint32_t scBase, ADC_SafetyCheckInst checkInst,
                           ADC_Select adcInst, ADC_ResultSelect adcResultInst)
{

    //
    // Configure safety checker instance
    //
    HW_WR_REG16(scBase + CSL_ADC_SAFETY_ADCRESSEL1 + ((uint16_t)checkInst),
    ((HW_RD_REG16(scBase + CSL_ADC_SAFETY_ADCRESSEL1 + ((uint16_t)checkInst)) &
    ~(CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_MASK |
    CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_MASK)) |
    ((uint16_t)adcInst << CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_SHIFT) |
    ((uint16_t)adcResultInst << CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_SHIFT)));

}

//*****************************************************************************
//
//! Configures the tolerance allowed between safety check results.
//!
//! \param scBase is the base address of the ADC safety checker module.
//! \param tolerance is the number of the start-of-conversion.
//!
//! This function configures the tolerance for the difference between
//! check result instances of the selected safety checker instance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setSafetyCheckerTolerance(uint32_t scBase, uint32_t tolerance)
{
    //
    // Check the arguments.
    //
    DebugP_assert(tolerance <= CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MASK);

    //
    // Set safety checker tolerance
    //
    HW_WR_REG32(scBase + CSL_ADC_SAFETY_TOLERANCE,
        (tolerance & CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MASK));
}

//*****************************************************************************
//
//! Returns the safety check result for the selected instance.
//!
//! \param scBase is the base address of the ADC safety checker module.
//! \param checkInst is the number of the start-of-conversion.
//!
//! This function returns the safety check result for the selected instance.
//!
//! The \e checkInst number is a value \b ADC_SAFETY_CHECKx where x is a number
//! from 1 to 2 specifying the safety result check instances available in the
//! safety checker instance specified by \e scBase.
//!
//!
//! \return Returns the safety checker result for the desired instance.
//
//*****************************************************************************
static inline uint32_t
ADC_getSafetyCheckerResult(uint32_t scBase, ADC_SafetyCheckInst checkInst)
{

    //
    // Returns the safety check result for the selected instance
    //
    return(HW_RD_REG32(scBase + CSL_ADC_SAFETY_CHECKRESULT1 +
        (uint16_t)checkInst) & CSL_ADC_SAFETY_CHECKRESULT1_RESULT_MASK);

}

//*****************************************************************************
//
//! Enables the safety checker event sources for selected instance.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkResult is the desired ADC Safety Checker result.
//! \param checkEvent is the desired event number.
//! \param checkerNumber is the number of the checker tile.
//!
//! This function enables the indicated safety checker event sources.
//!
//! The \e checkResult specifies the result of the OOT or OVF events. It should
//! be one of the following values.
//!
//! - \b ADC_SAFETY_CHECK_RESULT1
//! - \b ADC_SAFETY_CHECK_RESULT2
//! - \b ADC_SAFETY_CHECK_OOT
//!
//! The \e checkEvent is a value \b ADC_SAFETY_CHECK_EVENTx where x is
//! a number from 1 to 4 specifying the safety check events available in
//! the safety checker specified by \e scIntEvtBase.
//!
//! The \e checkerNumber number is a value \b ADC_SAFETY_CHECKERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableSafetyCheckEvt(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
            ADC_SafetyCheckEvent checkEvent, ADC_SafetyCheckResult checkResult)
{

    //
    // Enables the safety checker event source.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1 +
        (uint32_t)checkEvent + (uint32_t)checkResult,
        (HW_RD_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1 +
        (uint32_t)checkEvent + (uint32_t)checkResult) |
        (1UL << (uint32_t)checkerNumber)));

}

//*****************************************************************************
//
//! Disables the safety checker event sources for selected instance.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkResult is the desired ADC Safety Checker result.
//! \param checkEvent is the desired event number.
//! \param checkerNumber is the number of the checker tile.
//!
//! This function disables the indicated safety checker event sources.
//!
//! The \e checkResult specifies the result of the OOT or OVF events. It should
//! be one of the following values.
//!
//! - \b ADC_SAFETY_CHECK_RESULT1
//! - \b ADC_SAFETY_CHECK_RESULT2
//! - \b ADC_SAFETY_CHECK_OOT
//!
//! The \e checkEvent is a value \b ADC_SAFETY_CHECK_EVENTx where x is
//! a number from 1 to 4 specifying the safety check events available in
//! the safety checker specified by \e scIntEvtBase.
//!
//! The \e checkerNumber number is a value \b ADC_CHECKER_NUMBERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableSafetyCheckEvt(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
            ADC_SafetyCheckEvent checkEvent, ADC_SafetyCheckResult checkResult)
{

    //
    // Disables the safety checker event source.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1 +
        (uint32_t)checkEvent + (uint32_t)checkResult,
        (HW_RD_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1 +
        (uint32_t)checkEvent + (uint32_t)checkResult) &
        ~(1UL << (uint32_t)checkerNumber)));

}

//*****************************************************************************
//
//! Enables the safety checker interrupt sources for selected instance.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkResult is the desired ADC Safety Checker interrupt result.
//! \param checkerNumber is the number of the checker tile.
//!
//! This function enables the indicated safety checker interrupt sources.
//!
//! The \e checkResult specifies the result of the OOT or OVF events. It should
//! be one of the following values.
//!
//! - \b ADC_SAFETY_CHECK_RESULT1
//! - \b ADC_SAFETY_CHECK_RESULT2
//! - \b ADC_SAFETY_CHECK_OOT
//!
//! The \e checkerNumber number is a value \b ADC_CHECKER_NUMBERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableSafetyCheckInt(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
                         ADC_SafetyCheckResult checkResult)
{

    //
    // Enables the safety checker interrupt source.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKINTSEL1 +
        (uint32_t)checkResult,(HW_RD_REG32(scIntEvtBase +
        CSL_ADC_SAFETY_AGGR_CHECKINTSEL1 + (uint32_t)checkResult) |
        (1UL << (uint32_t)checkerNumber)));

}

//*****************************************************************************
//
//! Disables the safety checker interrupt sources for selected instance.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkResult is the desired ADC Safety Checker interrupt result.
//! \param checkerNumber is the number of the checker tile.
//!
//! This function disables the indicated safety checker interrupt sources.
//!
//! The \e checkResult specifies the result of the OOT or OVF events. It should
//! be one of the following values.
//!
//! - \b ADC_SAFETY_CHECK_RESULT1
//! - \b ADC_SAFETY_CHECK_RESULT2
//! - \b ADC_SAFETY_CHECK_OOT
//!
//! The \e checkerNumber number is a value \b ADC_CHECKER_NUMBERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableSafetyCheckInt(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
                          ADC_SafetyCheckResult checkResult)
{

    //
    // Enables the safety checker interrupt source.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKINTSEL1 +
        (uint32_t)checkResult,(HW_RD_REG32(scIntEvtBase +
        CSL_ADC_SAFETY_AGGR_CHECKINTSEL1 + (uint32_t)checkResult) &
        ~(1UL << (uint32_t)checkerNumber)));

}

//*****************************************************************************
//
//! Get the ADC safety checker OOT and OVF event status.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkerFlag is the desired ADC Safety Checker event status.
//! \param checkerNumber is the number of the checker tile.
//!
//! This function returns the event status for the OOT or OVF event.
//!
//! The \e checkerFlag specifies the ADC safety checker event status of
//! OOT or OVF events. It should be one of the following values.
//!
//! - \b ADC_SAFETY_CHECK_OOT_FLG
//! - \b ADC_SAFETY_CHECK_RES1OVF_FLG
//! - \b ADC_SAFETY_CHECK_RES2OVF_FLG
//!
//! The \e checkerNumber number is a value \b ADC_SAFETY_CHECKERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module specified by \e scIntEvtBase.
//!
//! \return Returns the status of the safety checker event module for
//! the selected instance.
//
//*****************************************************************************
static inline bool
ADC_getSafetyCheckStatus(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
                            ADC_SafetyCheckFlag checkerFlag)
{

    //
    // Get the specified safety checker event status.
    //
    return(HW_RD_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_OOTFLG +
        (uint32_t)checkerFlag) & (1U << (uint32_t)checkerNumber));

}

//*****************************************************************************
//
//! Clears the ADC safety checker OOT and OVF event status.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//! \param checkerFlag is the desired ADC Safety Checker event status.
//! \param checkerNumber is the number of the checker tile.
//!
//! This funtion clears the specified ADC Safety Checker event status so that
//! they no longer assert.
//!
//! \e checkerFlag takes one of the values \b ADC_SAFETY_CHECK_OOT_FLG,
//! \b ADC_SAFETY_CHECK_RES1OVF_FLG or \b ADC_SAFETY_CHECK_RES2OVF_FLG
//! to express which of the three event status of the ADC Safety Checker should
//! be cleared.
//!
//! The \e checkerNumber number is a value \b ADC_SAFETY_CHECKERx where x is
//! a number from 1 to 8 specifying which Checker Tile is to be selected in
//! the ADC module specified by \e scIntEvtBase.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearSafetyCheckStatus(uint32_t scIntEvtBase, ADC_Checker checkerNumber,
                            ADC_SafetyCheckFlag checkerFlag)
{

    //
    // Clear the specified safety checker event status.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_OOTFLGCLR +
    (uint32_t)checkerFlag, (1UL << (uint32_t)checkerNumber));

}

//*****************************************************************************
//
//! Get the ADC safety checker interrupt status.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//!
//! This function returns the interrupt status for the OOT or OVF event.
//!
//! \return Returns the status of the safety checker interrupt for
//! the selected instance.
//
//*****************************************************************************
static inline uint32_t
ADC_getSafetyCheckIntStatus(uint32_t scIntEvtBase)
{

    //
    // Get the specified safety checker interrupt status.
    //
    return(HW_RD_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKINTFLG));

}

//*****************************************************************************
//
//! Clears the ADC safety checker interrupt status.
//!
//! \param scIntEvtBase is the base address of ADC Safe Check INTEVT module.
//!
//! This funtion clears the specified ADC Safety Checker interrupt status
//! so that they no longer assert.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearSafetyCheckIntStatus(uint32_t scIntEvtBase)
{

    //
    // Clear the specified safety checker interrupt status.
    //
    HW_WR_REG32(scIntEvtBase + CSL_ADC_SAFETY_AGGR_CHECKINTFLGCLR, 1U);

}

//*****************************************************************************
//
//! Configures the trigger repeater mode select.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param mode is the repeater mode.
//!
//! This function configures desired ADC trigger repeater mode that corresponds
//! to \e mode to select either oversampling or undersampling mode.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterMode(uint32_t base, uint32_t repInstance, ADC_RepMode mode)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Set the specified repeater trigger source to modify.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1CTL,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
        ~CSL_ADC_REP1CTL_MODE_MASK) | (uint32_t)mode));

}

//*****************************************************************************
//
//! Get the trigger repeater active mode status.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//!
//! This function returns the status of the ADC trigger repeater mode for
//! the selected repeater instance.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return Returns the status of the trigger repeater active mode.
//
//*****************************************************************************
static inline bool
ADC_triggerRepeaterActiveMode(uint32_t base, uint32_t repInstance)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // get the specified repeater trigger active mode status.
    //
    return(HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
        (1U << CSL_ADC_REP1CTL_ACTIVEMODE_SHIFT));

}

//*****************************************************************************
//
//! Get the trigger repeater module busy indication.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//!
//! This function returns the status of the ADC trigger repeater module for
//! the selected repeater instance.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return Returns the staus of the trigger repeater module busy.
//
//*****************************************************************************
static inline bool
ADC_triggerRepeaterModuleBusy(uint32_t base, uint32_t repInstance)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // get the specified repeater trigger active mode status.
    //
    return(HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
        (1U << CSL_ADC_REP1CTL_MODULEBUSY_SHIFT));

}

//*****************************************************************************
//
//! Configures the trigger source of the trigger repeater.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param trigger the source to modify via oversampling or undersampling.
//!
//! This function configures ADC trigger repeater by selecting the trigger
//! source passed into \e trigger to modify via oversampling or undersampling.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterSelect(uint32_t base, uint16_t repInstance,
                          ADC_Trigger trigger)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Set the specified repeater trigger source to modify.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1CTL,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
        ~CSL_ADC_REP1CTL_TRIGGER_MASK) |
        ((uint32_t)trigger << CSL_ADC_REP1CTL_TRIGGER_SHIFT)));

}

//*****************************************************************************
//
//! Configures the trigger repeater syncin source.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param syncInput is the desired sync event to reset all counters.
//!
//! This function configures desired ADC trigger repeater sync event that
//! corresponds to \e syncInput to reset all counters and
//! any pending repeated triggers are discarded.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterSyncIn(uint32_t base, uint16_t repInstance,
                          ADC_SyncInput syncInput)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Set the specified trigger sync input.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1CTL,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
        ~CSL_ADC_REP1CTL_SYNCINSEL_MASK) |
        ((uint32_t)syncInput << CSL_ADC_REP1CTL_SYNCINSEL_SHIFT)));

}

//*****************************************************************************
//
//!
//! Forces software sync for the trigger repeater block.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//!
//! This function forces the software sync for the trigger repeater block.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceRepeaterTriggerSync(uint32_t base, uint16_t repInstance)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Force software sync for the trigger repeater block.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1CTL,
        (HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) |
        CSL_ADC_REP1CTL_SWSYNC_MASK));

}

//*****************************************************************************
//
//! Configures the trigger repeater count.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param repCount is the desired trigger count to be selected.
//!
//! This function indicates the number of repeated triggers passed into
//! \e repCount and remaining triggers to be generated/supressed.
//! In oversampling mode, the \e repCount parameter is the number of desired
//! repeated triggers to be generated. It should be a value between 0 to 127
//! where (repCount + 1) triggers will be generated. For example, when
//! \e repCount = 2, 3 triggers will be generated on receiving corresponding
//! REPxCTL.TRIGSEL. In unversampling mode, the \e repCount parameter is
//! the number of desired triggers to be supressed. It should be a value between
//! 0 to 127 where 1 out of (repCount + 1) triggers received will be passed
//! through. For Example, when \e repCount = 2, 1 out of 3 triggers will be
//! generated.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterCount(uint32_t base, uint16_t repInstance,
                          uint16_t repCount)
{
    uint32_t regOffset;
    //
    // Check the arguments.
    //
    DebugP_assert(repCount <= 127U);

    regOffset = base + (repInstance * (ADC_REPxN_STEP));

    //
    // Configure repeater count.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1N,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1N) &
        ~CSL_ADC_REP1N_NSEL_MASK) | repCount));

}

//*****************************************************************************
//
//! Configures the trigger repeater phase.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param repPhase indicates the repeater trigger phase delay
//! in sysclk cycles.
//!
//! This function configures the phase delay that corresponds to \e repPhase
//! by defining the number of sysclk to delay the selected trigger.
//! The \e repPhase parameter should be a value between 0 and 65535 inclusive.
//! For example, passing a 2 to the \e offset parameter will delay the trigger
//! by 2 sysclks and passing 0 will pass through the trigger without delay.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterPhase(uint32_t base, uint16_t repInstance,
                          uint16_t repPhase)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxPHASE_STEP));

    //
    // Configure repeater phase.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1PHASE,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1PHASE) &
        ~CSL_ADC_REP1PHASE_PHASE_MASK) | repPhase));

}

//*****************************************************************************
//
//! Configures the trigger repeater spread.
//!
//! \param base is the base address of the ADC module.
//! \param repInstance is the repeater instance.
//! \param repSpread is the desired trigger spread in sysclk cycle.
//!
//! This function configures the spread time by setting \e repSpread
//! to number of sysclk desired between triggers. In oversampling mode,
//! the \e repSpread parameter is the minimum number of sysclks to wait before
//! creating the next repeated trigger to the ADC. It should be a value between
//! 0 and 65535 inclusive. For example, passing a 2 to the \e offset parameter
//! will create at least 2 sysclk time between repeated triggers.
//!
//! The \e repInstance is the repeater instance to be configured. Valid values
//! for this parameter can be referred from the enum \e ADC_RepInstance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_triggerRepeaterSpread(uint32_t base, uint16_t repInstance,
                          uint16_t repSpread)
{
    uint32_t regOffset;

    regOffset = base + (repInstance * (ADC_REPxSPREAD_STEP));

    //
    // Configure repeater spread.
    //
    HW_WR_REG32(regOffset + CSL_ADC_REP1SPREAD,
        ((HW_RD_REG32(regOffset + CSL_ADC_REP1SPREAD) &
        ~CSL_ADC_REP1SPREAD_SPREAD_MASK) | repSpread));

}

//
//! Configures the analog-to-digital converter resolution and signal mode.
//!
//! \param base is the base address of the ADC module.
//! \param resolution is the resolution of the converter (12 bits).
//! \param signalMode is the input signal mode of the converter.
//!
//! This function configures the ADC module's conversion resolution and input
//! signal mode and ensures that the corresponding trims are loaded.
//!
//! The \e resolution parameter specifies the resolution of the conversion.
//! It can be 12-bit specified by \b ADC_RESOLUTION_12BIT
//!
//! The \e signalMode parameter specifies the signal mode. In single-ended
//! mode, which is indicated by \b ADC_MODE_SINGLE_ENDED, the input voltage is
//! sampled on a single pin referenced to VREFLO. In differential mode, which
//! is indicated by \b ADC_MODE_DIFFERENTIAL, the input voltage to the
//! converter is sampled on a pair of input pins, a positive and a negative.
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setMode(uint32_t base, ADC_Resolution resolution,
            ADC_SignalMode signalMode);

//*****************************************************************************
//
//! Sets the windowed trip limits for a PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param tripHiLimit is the value is the digital comparator trip high limit.
//! \param tripLoLimit is the value is the digital comparator trip low limit.
//!
//! This function sets the windowed trip limits for a PPB. These values set
//! the digital comparator so that when one of the values is exceeded, either a
//! high or low trip event will occur.
//!
//! The \e ppbNumber is a value \b ADC_PPB_NUMBERX where X is a value from 1 to
//! 4 inclusive that identifies a PPB to be configured.
//!
//! In 12-bit mode, only bits 12:0 will be compared against bits 12:0
//! of the PPB result.
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // ADC_V1_H_
