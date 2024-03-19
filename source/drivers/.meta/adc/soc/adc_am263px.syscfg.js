let ADC_ClkPrescale = [
	{ name: "ADC_CLK_DIV_1_0", displayName: "ADCCLK = (input clock) / 1.0" },
	{ name: "ADC_CLK_DIV_2_0", displayName: "ADCCLK = (input clock) / 2.0" },
	{ name: "ADC_CLK_DIV_2_5", displayName: "ADCCLK = (input clock) / 2.5" },
	{ name: "ADC_CLK_DIV_3_0", displayName: "ADCCLK = (input clock) / 3.0" },
	{ name: "ADC_CLK_DIV_3_5", displayName: "ADCCLK = (input clock) / 3.5" },
	{ name: "ADC_CLK_DIV_4_0", displayName: "ADCCLK = (input clock) / 4.0" },
	{ name: "ADC_CLK_DIV_4_5", displayName: "ADCCLK = (input clock) / 4.5" },
	{ name: "ADC_CLK_DIV_5_0", displayName: "ADCCLK = (input clock) / 5.0" },
	{ name: "ADC_CLK_DIV_5_5", displayName: "ADCCLK = (input clock) / 5.5" },
	{ name: "ADC_CLK_DIV_6_0", displayName: "ADCCLK = (input clock) / 6.0" },
	{ name: "ADC_CLK_DIV_6_5", displayName: "ADCCLK = (input clock) / 6.5" },
	{ name: "ADC_CLK_DIV_7_0", displayName: "ADCCLK = (input clock) / 7.0" },
	{ name: "ADC_CLK_DIV_7_5", displayName: "ADCCLK = (input clock) / 7.5" },
	{ name: "ADC_CLK_DIV_8_0", displayName: "ADCCLK = (input clock) / 8.0" },
	{ name: "ADC_CLK_DIV_8_5", displayName: "ADCCLK = (input clock) / 8.5" },
]
let ADC_Resolution = [
	{ name: "ADC_RESOLUTION_12BIT", displayName: "12-bit conversion resolution" },
]
let ADC_SignalMode = [
	{ name: "ADC_MODE_SINGLE_ENDED", displayName: "Sample on single pin with VREFLO" },
	{ name: "ADC_MODE_DIFFERENTIAL", displayName: "Sample on pair of pins" },
]
let ADC_Trigger = [
    { name: "ADC_TRIGGER_SW_ONLY", displayName: "Software only" },
    { name: "ADC_TRIGGER_RTI0", displayName: "RTI Timer 0" },
    { name: "ADC_TRIGGER_RTI1", displayName: "RTI Timer 1" },
    { name: "ADC_TRIGGER_RTI2", displayName: "RTI Timer 2" },
    { name: "ADC_TRIGGER_RTI3", displayName: "RTI Timer 3" },
    { name: "ADC_TRIGGER_INPUT_XBAR_OUT5", displayName: "InputXBar.Out[5]" },
    { name: "ADC_TRIGGER_EPWM0_SOCA", displayName: "ePWM0, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM0_SOCB", displayName: "ePWM0, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM1_SOCA", displayName: "ePWM1, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM1_SOCB", displayName: "ePWM1, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM2_SOCA", displayName: "ePWM2, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM2_SOCB", displayName: "ePWM2, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM3_SOCA", displayName: "ePWM3, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM3_SOCB", displayName: "ePWM3, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM4_SOCA", displayName: "ePWM4, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM4_SOCB", displayName: "ePWM4, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM5_SOCA", displayName: "ePWM5, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM5_SOCB", displayName: "ePWM5, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM6_SOCA", displayName: "ePWM6, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM6_SOCB", displayName: "ePWM6, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM7_SOCA", displayName: "ePWM7, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM7_SOCB", displayName: "ePWM7, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM8_SOCA", displayName: "ePWM8, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM8_SOCB", displayName: "ePWM8, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM9_SOCA", displayName: "ePWM9, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM9_SOCB", displayName: "ePWM9, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM10_SOCA", displayName: "ePWM10, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM10_SOCB", displayName: "ePWM10, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM11_SOCA", displayName: "ePWM11, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM11_SOCB", displayName: "ePWM11, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM12_SOCA", displayName: "ePWM12, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM12_SOCB", displayName: "ePWM12, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM13_SOCA", displayName: "ePWM13, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM13_SOCB", displayName: "ePWM13, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM14_SOCA", displayName: "ePWM14, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM14_SOCB", displayName: "ePWM14, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM15_SOCA", displayName: "ePWM15, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM15_SOCB", displayName: "ePWM15, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM16_SOCA", displayName: "ePWM16, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM16_SOCB", displayName: "ePWM16, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM17_SOCA", displayName: "ePWM17, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM17_SOCB", displayName: "ePWM17, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM18_SOCA", displayName: "ePWM18, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM18_SOCB", displayName: "ePWM18, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM19_SOCA", displayName: "ePWM19, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM19_SOCB", displayName: "ePWM19, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM20_SOCA", displayName: "ePWM20, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM20_SOCB", displayName: "ePWM20, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM21_SOCA", displayName: "ePWM21, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM21_SOCB", displayName: "ePWM21, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM22_SOCA", displayName: "ePWM22, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM22_SOCB", displayName: "ePWM22, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM23_SOCA", displayName: "ePWM23, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM23_SOCB", displayName: "ePWM23, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM24_SOCA", displayName: "ePWM24, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM24_SOCB", displayName: "ePWM24, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM25_SOCA", displayName: "ePWM25, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM25_SOCB", displayName: "ePWM25, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM26_SOCA", displayName: "ePWM26, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM26_SOCB", displayName: "ePWM26, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM27_SOCA", displayName: "ePWM27, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM27_SOCB", displayName: "ePWM27, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM28_SOCA", displayName: "ePWM28, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM28_SOCB", displayName: "ePWM28, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM29_SOCA", displayName: "ePWM29, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM29_SOCB", displayName: "ePWM29, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM30_SOCA", displayName: "ePWM30, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM30_SOCB", displayName: "ePWM30, ADCSOCB" },
    { name: "ADC_TRIGGER_EPWM31_SOCA", displayName: "ePWM31, ADCSOCA" },
    { name: "ADC_TRIGGER_EPWM31_SOCB", displayName: "ePWM31, ADCSOCB" },
    { name: "ADC_TRIGGER_ECAP0_SOCEVT", displayName: "eCAP0, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP1_SOCEVT", displayName: "eCAP1, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP2_SOCEVT", displayName: "eCAP2, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP3_SOCEVT", displayName: "eCAP3, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP4_SOCEVT", displayName: "eCAP4, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP5_SOCEVT", displayName: "eCAP5, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP6_SOCEVT", displayName: "eCAP6, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP7_SOCEVT", displayName: "eCAP7, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP8_SOCEVT", displayName: "eCAP8, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP9_SOCEVT", displayName: "eCAP9, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP10_SOCEVT", displayName: "eCAP10, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP11_SOCEVT", displayName: "eCAP11, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP12_SOCEVT", displayName: "eCAP12, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP13_SOCEVT", displayName: "eCAP13, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP14_SOCEVT", displayName: "eCAP14, SOCEVT" },
    { name: "ADC_TRIGGER_ECAP15_SOCEVT", displayName: "eCAP15, SOCEVT" },
    { name: "ADC_TRIGGER_RTI4", displayName: "RTI Timer 4" },
    { name: "ADC_TRIGGER_RTI5", displayName: "RTI Timer 5" },
    { name: "ADC_TRIGGER_RTI6", displayName: "RTI Timer 6" },
    { name: "ADC_TRIGGER_RTI7", displayName: "RTI Timer 7" },
	{ name: "ADC_TRIGGER_REPEATER1", displayName: "Repeater 1" },
	{ name: "ADC_TRIGGER_REPEATER2", displayName: "Repeater 2" },
]
let ADC_Channel = [
	{ name: "ADC_CH_ADCIN0", displayName: "single-ended, ADCIN0" },
	{ name: "ADC_CH_ADCIN1", displayName: "single-ended, ADCIN1" },
	{ name: "ADC_CH_ADCIN2", displayName: "single-ended, ADCIN2" },
	{ name: "ADC_CH_ADCIN3", displayName: "single-ended, ADCIN3" },
	{ name: "ADC_CH_ADCIN4", displayName: "single-ended, ADCIN4" },
	{ name: "ADC_CH_ADCIN5", displayName: "single-ended, ADCIN5" },
    { name: "ADC_CH_CAL0", displayName: "single-ended, CAL0" },
	{ name: "ADC_CH_CAL1", displayName: "single-ended, CAL1" },
	{ name: "ADC_CH_ADCIN0_ADCIN1", displayName: "differential, ADCIN0 and ADCIN1" },
	{ name: "ADC_CH_ADCIN1_ADCIN0", displayName: "differential, ADCIN1 and ADCIN0" },
	{ name: "ADC_CH_ADCIN2_ADCIN3", displayName: "differential, ADCIN2 and ADCIN3" },
	{ name: "ADC_CH_ADCIN3_ADCIN2", displayName: "differential, ADCIN3 and ADCIN2" },
	{ name: "ADC_CH_ADCIN4_ADCIN5", displayName: "differential, ADCIN4 and ADCIN5" },
    { name: "ADC_CH_ADCIN5_ADCIN4", displayName: "differential, ADCIN5 and ADCIN4" },
    { name: "ADC_CH_CAL0_CAL1", displayName: "single-ended, CAL0 and CAL1" },
]
let ADC_R_Channel = [
	{ name: "ADC_CH_ADCIN0", displayName: "single-ended, ADC_R_IN0" },
	{ name: "ADC_CH_ADCIN1", displayName: "single-ended, ADC_R_IN1" },
	{ name: "ADC_CH_ADCIN2", displayName: "single-ended, ADC_R_IN2" },
	{ name: "ADC_CH_ADCIN3", displayName: "single-ended, ADC_R_IN3" },
	{ name: "ADC_CH_ADCIN4", displayName: "single-ended, CAL Channel 2" },
	{ name: "ADC_CH_ADCIN5", displayName: "single-ended, CAL Channel 3" },
	{ name: "ADC_CH_ADCIN0_ADCIN1", displayName: "differential, ADC_R_IN0 and ADC_R_IN1" },
	{ name: "ADC_CH_ADCIN1_ADCIN0", displayName: "differential, ADC_R_IN1 and ADC_R_IN0" },
	{ name: "ADC_CH_ADCIN2_ADCIN3", displayName: "differential, ADC_R_IN2 and ADC_R_IN3" },
	{ name: "ADC_CH_ADCIN3_ADCIN2", displayName: "differential, ADC_R_IN3 and ADC_R_IN2" },
	{ name: "ADC_CH_ADCIN4_ADCIN5", displayName: "differential, CAL Channel 2 and CAL Channel 3" },
    { name: "ADC_CH_ADCIN5_ADCIN4", displayName: "differential, CAL Channel 3 and CAL Channel 2" },

]
let ADC_PulseMode = [
	{ name: "ADC_PULSE_END_OF_ACQ_WIN", displayName: "Occurs at the end of the acquisition window" },
	{ name: "ADC_PULSE_END_OF_CONV", displayName: "Occurs at the end of the conversion" },
]
let ADC_IntNumber = [
	{ name: "ADC_INT_NUMBER1", displayName: "ADCINT1 Interrupt" },
	{ name: "ADC_INT_NUMBER2", displayName: "ADCINT2 Interrupt" },
	{ name: "ADC_INT_NUMBER3", displayName: "ADCINT3 Interrupt" },
	{ name: "ADC_INT_NUMBER4", displayName: "ADCINT4 Interrupt" },
]
let ADC_PPBNumber = [
	{ name: "ADC_PPB_NUMBER1", displayName: "Post-processing block 1" },
	{ name: "ADC_PPB_NUMBER2", displayName: "Post-processing block 2" },
	{ name: "ADC_PPB_NUMBER3", displayName: "Post-processing block 3" },
	{ name: "ADC_PPB_NUMBER4", displayName: "Post-processing block 4" },
]
let ADC_SOCNumber = [
	{ name: "ADC_SOC_NUMBER0", displayName: "SOC/EOC number 0" },
	{ name: "ADC_SOC_NUMBER1", displayName: "SOC/EOC number 1" },
	{ name: "ADC_SOC_NUMBER2", displayName: "SOC/EOC number 2" },
	{ name: "ADC_SOC_NUMBER3", displayName: "SOC/EOC number 3" },
	{ name: "ADC_SOC_NUMBER4", displayName: "SOC/EOC number 4" },
	{ name: "ADC_SOC_NUMBER5", displayName: "SOC/EOC number 5" },
	{ name: "ADC_SOC_NUMBER6", displayName: "SOC/EOC number 6" },
	{ name: "ADC_SOC_NUMBER7", displayName: "SOC/EOC number 7" },
	{ name: "ADC_SOC_NUMBER8", displayName: "SOC/EOC number 8" },
	{ name: "ADC_SOC_NUMBER9", displayName: "SOC/EOC number 9" },
	{ name: "ADC_SOC_NUMBER10", displayName: "SOC/EOC number 10" },
	{ name: "ADC_SOC_NUMBER11", displayName: "SOC/EOC number 11" },
	{ name: "ADC_SOC_NUMBER12", displayName: "SOC/EOC number 12" },
	{ name: "ADC_SOC_NUMBER13", displayName: "SOC/EOC number 13" },
	{ name: "ADC_SOC_NUMBER14", displayName: "SOC/EOC number 14" },
	{ name: "ADC_SOC_NUMBER15", displayName: "SOC/EOC number 15" },
]
let ADC_IntSOCTrigger = [
	{ name: "ADC_INT_SOC_TRIGGER_NONE", displayName: "No ADCINT will trigger the SOC" },
	{ name: "ADC_INT_SOC_TRIGGER_ADCINT1", displayName: "ADCINT1 will trigger the SOC" },
	{ name: "ADC_INT_SOC_TRIGGER_ADCINT2", displayName: "ADCINT2 will trigger the SOC" },
]
let ADC_PriorityMode = [
	{ name: "ADC_PRI_ALL_ROUND_ROBIN", displayName: "Round robin mode is used for all" },
	{ name: "ADC_PRI_SOC0_HIPRI", displayName: "SOC 0 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC1_HIPRI", displayName: "SOC 0-1 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC2_HIPRI", displayName: "SOC 0-2 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC3_HIPRI", displayName: "SOC 0-3 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC4_HIPRI", displayName: "SOC 0-4 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC5_HIPRI", displayName: "SOC 0-5 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC6_HIPRI", displayName: "SOC 0-6 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC7_HIPRI", displayName: "SOC 0-7 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC8_HIPRI", displayName: "SOC 0-8 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC9_HIPRI", displayName: "SOC 0-9 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC10_HIPRI", displayName: "SOC 0-10 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC11_HIPRI", displayName: "SOC 0-11 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC12_HIPRI", displayName: "SOC 0-12 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC13_HIPRI", displayName: "SOC 0-13 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC14_HIPRI", displayName: "SOC 0-14 hi pri, SOC15 in round robin" },
	{ name: "ADC_PRI_ALL_HIPRI", displayName: "All priorities based on SOC number" },
]
let ADC_OSDetectMode = [
	{ name: "ADC_OSDETECT_MODE_DISABLED", displayName: "Open/Shorts detection cir-" },
	{ name: "ADC_OSDETECT_MODE_VSSA", displayName: "O/S DC is enabled at zero" },
	{ name: "ADC_OSDETECT_MODE_VDDA", displayName: "O/S DC is enabled at full" },
	{ name: "ADC_OSDETECT_MODE_5BY12_VDDA", displayName: "O/S DC is enabled at 5/12" },
	{ name: "ADC_OSDETECT_MODE_7BY12_VDDA", displayName: "O/S DC is enabled at 7/12" },
	{ name: "ADC_OSDETECT_MODE_5K_PULLDOWN_TO_VSSA", displayName: "O/S DC is enabled at 5K" },
	{ name: "ADC_OSDETECT_MODE_5K_PULLUP_TO_VDDA", displayName: "O/S DC is enabled at 5K" },
	{ name: "ADC_OSDETECT_MODE_7K_PULLDOWN_TO_VSSA", displayName: "O/S DC is enabled at 7K" },
]
let ADC_OffsetTrim = [
	{ name: "ADC_OFFSET_TRIM_COMMON", displayName: "Common Trim register for all" },
	{ name: "ADC_OFFSET_TRIM_INDIVIDUAL", displayName: "Individual Trim registers for" },
]
let ADC_SyncInput = [
	{ name: "ADC_SYNCIN_DISABLE", displayName: "ADC Syncin is disabled" },
    { name: "ADC_SYNCIN_EPWM0SYNCOUT", displayName: "ADC Syncin is EPWM0SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM1SYNCOUT", displayName: "ADC Syncin is EPWM1SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM2SYNCOUT", displayName: "ADC Syncin is EPWM2SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM3SYNCOUT", displayName: "ADC Syncin is EPWM3SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM4SYNCOUT", displayName: "ADC Syncin is EPWM4SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM5SYNCOUT", displayName: "ADC Syncin is EPWM5SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM6SYNCOUT", displayName: "ADC Syncin is EPWM6SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM7SYNCOUT", displayName: "ADC Syncin is EPWM7SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM8SYNCOUT", displayName: "ADC Syncin is EPWM8SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM9SYNCOUT", displayName: "ADC Syncin is EPWM9SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM10SYNCOUT", displayName: "ADC Syncin is EPWM10SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM11SYNCOUT", displayName: "ADC Syncin is EPWM11SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM12SYNCOUT", displayName: "ADC Syncin is EPWM12SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM13SYNCOUT", displayName: "ADC Syncin is EPWM13SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM14SYNCOUT", displayName: "ADC Syncin is EPWM14SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM15SYNCOUT", displayName: "ADC Syncin is EPWM15SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM16SYNCOUT", displayName: "ADC Syncin is EPWM16SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM17SYNCOUT", displayName: "ADC Syncin is EPWM17SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM18SYNCOUT", displayName: "ADC Syncin is EPWM18SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM19SYNCOUT", displayName: "ADC Syncin is EPWM19SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM20SYNCOUT", displayName: "ADC Syncin is EPWM20SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM21SYNCOUT", displayName: "ADC Syncin is EPWM21SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM22SYNCOUT", displayName: "ADC Syncin is EPWM22SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM23SYNCOUT", displayName: "ADC Syncin is EPWM23SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM24SYNCOUT", displayName: "ADC Syncin is EPWM24SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM25SYNCOUT", displayName: "ADC Syncin is EPWM25SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM26SYNCOUT", displayName: "ADC Syncin is EPWM26SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM27SYNCOUT", displayName: "ADC Syncin is EPWM27SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM28SYNCOUT", displayName: "ADC Syncin is EPWM28SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM29SYNCOUT", displayName: "ADC Syncin is EPWM29SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM30SYNCOUT", displayName: "ADC Syncin is EPWM30SYNCOUT" },
	{ name: "ADC_SYNCIN_EPWM31SYNCOUT", displayName: "ADC Syncin is EPWM31SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP0YNCOUT", displayName: "ADC Syncin is ECAP0YNCOUT" },
	{ name: "ADC_SYNCIN_ECAP1YNCOUT", displayName: "ADC Syncin is ECAP1YNCOUT" },
	{ name: "ADC_SYNCIN_ECAP2SYNCOUT", displayName: "ADC Syncin is ECAP2SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP3SYNCOUT", displayName: "ADC Syncin is ECAP3SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP4SYNCOUT", displayName: "ADC Syncin is ECAP4SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP5SYNCOUT", displayName: "ADC Syncin is ECAP5SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP6SYNCOUT", displayName: "ADC Syncin is ECAP6SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP7SYNCOUT", displayName: "ADC Syncin is ECAP7SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP8SYNCOUT", displayName: "ADC Syncin is ECAP8SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP9SYNCOUT", displayName: "ADC Syncin is ECAP9SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP10SYNCOUT", displayName: "ADC Syncin is ECAP10SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP11SYNCOUT", displayName: "ADC Syncin is ECAP11SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP12SYNCOUT", displayName: "ADC Syncin is ECAP12SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP13SYNCOUT", displayName: "ADC Syncin is ECAP13SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP14SYNCOUT", displayName: "ADC Syncin is ECAP14SYNCOUT" },
	{ name: "ADC_SYNCIN_ECAP15SYNCOUT", displayName: "ADC Syncin is ECAP15SYNCOUT" },
	{ name: "ADC_SYNCIN_INPUTXBAROUTPUT6", displayName: "ADC Syncin is INPUTXBAROUTPUT6" },
	{ name: "ADC_SYNCIN_INPUTXBAROUTPUT7", displayName: "ADC Syncin is INPUTXBAROUTPUT7" },
	{ name: "ADC_SYNCIN_CPSW_CTPS_SYNC", displayName: "ADC Syncin is CPSW_CTPS_SYNC" },
]
let ADC_PPBIntSrcSelect = [
	{ name: "ADC_PPB_OS_INT_1", displayName: "PCount generates PPB interrupt" },
	{ name: "ADC_PPB_OS_INT_2", displayName: "PCount/Sync generates PPB interrupt" },
]
let ADC_ExtChannel = [
	{ name: "ADC_CH_ADCINX_0", displayName: "ADCINX.0 is converted" },
	{ name: "ADC_CH_ADCINX_1", displayName: "ADCINX.1 is converted" },
	{ name: "ADC_CH_ADCINX_2", displayName: "ADCINX.2 is converted" },
	{ name: "ADC_CH_ADCINX_3", displayName: "ADCINX.3 is converted" },
]
let ADC_IntTrigger = [
	{ name: "ADC_SOC_NUMBER0", displayName: "SOC/EOC0" },
	{ name: "ADC_SOC_NUMBER1", displayName: "SOC/EOC1" },
	{ name: "ADC_SOC_NUMBER2", displayName: "SOC/EOC2" },
	{ name: "ADC_SOC_NUMBER3", displayName: "SOC/EOC3" },
	{ name: "ADC_SOC_NUMBER4", displayName: "SOC/EOC4" },
	{ name: "ADC_SOC_NUMBER5", displayName: "SOC/EOC5" },
	{ name: "ADC_SOC_NUMBER6", displayName: "SOC/EOC6" },
	{ name: "ADC_SOC_NUMBER7", displayName: "SOC/EOC7" },
	{ name: "ADC_SOC_NUMBER8", displayName: "SOC/EOC8" },
	{ name: "ADC_SOC_NUMBER9", displayName: "SOC/EOC9" },
	{ name: "ADC_SOC_NUMBER10", displayName: "SOC/EOC10" },
	{ name: "ADC_SOC_NUMBER11", displayName: "SOC/EOC11" },
	{ name: "ADC_SOC_NUMBER12", displayName: "SOC/EOC12" },
	{ name: "ADC_SOC_NUMBER13", displayName: "SOC/EOC13" },
	{ name: "ADC_SOC_NUMBER14", displayName: "SOC/EOC14" },
	{ name: "ADC_SOC_NUMBER15", displayName: "SOC/EOC15" },
	{ name: "ADC_INT_TRIGGER_OSINT1", displayName: "OSINT1" },
	{ name: "ADC_INT_TRIGGER_OSINT2", displayName: "OSINT2" },
	{ name: "ADC_INT_TRIGGER_OSINT3", displayName: "OSINT3" },
	{ name: "ADC_INT_TRIGGER_OSINT4", displayName: "OSINT4" },
]
let ADC_PPBCompSource = [
	{ name: "ADC_PPB_COMPSOURCE_RESULT", displayName: "PPB compare source is ADCRESULT" },
	{ name: "ADC_PPB_COMPSOURCE_PSUM", displayName: "PPB compare source is PSUM" },
	{ name: "ADC_PPB_COMPSOURCE_SUM", displayName: "PPB compare source is SUM" },
]
let ADC_Select = [
	{ name: "ADC_0", displayName: "Select ADC0 instance" },
	{ name: "ADC_1", displayName: "Select ADC1 instance" },
	{ name: "ADC_2", displayName: "Select ADC2 instance" },
    { name: "ADC_3", displayName: "Select ADC3 instance" },
	{ name: "ADC_4", displayName: "Select ADC4 instance" },
]
let ADC_ResultSelect = [
	{ name: "ADC_RESULT0", displayName: "Select ADC Result 0" },
	{ name: "ADC_RESULT1", displayName: "Select ADC Result 1" },
	{ name: "ADC_RESULT2", displayName: "Select ADC Result 2" },
	{ name: "ADC_RESULT3", displayName: "Select ADC Result 3" },
	{ name: "ADC_RESULT4", displayName: "Select ADC Result 4" },
	{ name: "ADC_RESULT5", displayName: "Select ADC Result 5" },
	{ name: "ADC_RESULT6", displayName: "Select ADC Result 6" },
	{ name: "ADC_RESULT7", displayName: "Select ADC Result 7" },
	{ name: "ADC_RESULT8", displayName: "Select ADC Result 8" },
	{ name: "ADC_RESULT9", displayName: "Select ADC Result 9" },
	{ name: "ADC_RESULT10", displayName: "Select ADC Result 10" },
	{ name: "ADC_RESULT11", displayName: "Select ADC Result 11" },
	{ name: "ADC_RESULT12", displayName: "Select ADC Result 12" },
	{ name: "ADC_RESULT13", displayName: "Select ADC Result 13" },
	{ name: "ADC_RESULT14", displayName: "Select ADC Result 14" },
	{ name: "ADC_RESULT15", displayName: "Select ADC Result 15" },
]
let ADC_SafetyCheckerInput = [
	{ name: "ADC_SAFETY_CHECKER_INPUT_DISABLE", displayName: "Safety checker i/p disabled" },
	{ name: "ADC_SAFETY_CHECKER_INPUT_SOCx", displayName: "Safety checker i/p is SOCx" },
	{ name: "ADC_SAFETY_CHECKER_INPUT_PPBx", displayName: "Safety checker i/p is PPBx" },
	{ name: "ADC_SAFETY_CHECKER_INPUT_PPBSUMx", displayName: "Safety checker i/p is PPBSUMx" },
]
let ADC_SafetyCheckInst = [
	{ name: "ADC_SAFETY_CHECK1", displayName: "Safety Check Result 1" },
	{ name: "ADC_SAFETY_CHECK2", displayName: "Safety Check Result 2" },
]
let ADC_SafetyCheckEvent = [
	{ name: "ADC_SAFETY_CHECK_EVENT1", displayName: "Safety Check Event 1" },
	{ name: "ADC_SAFETY_CHECK_EVENT2", displayName: "Safety Check Event 2" },
	{ name: "ADC_SAFETY_CHECK_EVENT3", displayName: "Safety Check Event 3" },
	{ name: "ADC_SAFETY_CHECK_EVENT4", displayName: "Safety Check Event 4" },
]
let ADC_SafetyCheckResult = [
	{ name: "ADC_SAFETY_CHECK_RES1OVF", displayName: "Safety Check Result1 Overflow" },
	{ name: "ADC_SAFETY_CHECK_RES2OVF", displayName: "Safety Check Result2 Overflow" },
	{ name: "ADC_SAFETY_CHECK_OOT", displayName: "Safety Check OOT" },
]
let ADC_Checker = [
	{ name: "ADC_SAFETY_CHECKER1", displayName: "Safety Checker1" },
	{ name: "ADC_SAFETY_CHECKER2", displayName: "Safety Checker2" },
	{ name: "ADC_SAFETY_CHECKER3", displayName: "Safety Checker3" },
	{ name: "ADC_SAFETY_CHECKER4", displayName: "Safety Checker4" },
	{ name: "ADC_SAFETY_CHECKER5", displayName: "Safety Checker5" },
	{ name: "ADC_SAFETY_CHECKER6", displayName: "Safety Checker6" },
	{ name: "ADC_SAFETY_CHECKER7", displayName: "Safety Checker7" },
	{ name: "ADC_SAFETY_CHECKER8", displayName: "Safety Checker8" },
    { name: "ADC_SAFETY_CHECKER9", displayName: "Safety Checker9" },
	{ name: "ADC_SAFETY_CHECKER10", displayName: "Safety Checker10" },
	{ name: "ADC_SAFETY_CHECKER11", displayName: "Safety Checker11" },
	{ name: "ADC_SAFETY_CHECKER12", displayName: "Safety Checker12" },
]
let ADC_SafetyCheckFlag = [
	{ name: "ADC_SAFETY_CHECK_OOT_FLG", displayName: "Safety Check Out-of-Tolerance Flag" },
	{ name: "ADC_SAFETY_CHECK_RES1OVF_FLG", displayName: "Safety Check Result1 Overflow Flag" },
	{ name: "ADC_SAFETY_CHECK_RES2OVF_FLG", displayName: "Safety Check Result2 Overflow Flag" },
]
let ADC_RepInstance = [
	{ name: "ADC_REPINST1", displayName: "Select ADC repeater instance 1" },
	{ name: "ADC_REPINST2", displayName: "Select ADC repeater instance 2" },
]
let ADC_RepMode = [
	{ name: "ADC_REPMODE_OVERSAMPLING", displayName: "ADC repeater mode is oversampling" },
	{ name: "ADC_REPMODE_UNDERSAMPLING", displayName: "ADC repeater mode is undersampling" },
]
let ADC_EVT = [
	{ name: "ADC_EVT_TRIPHI", displayName: "Trip High Event" },
	{ name: "ADC_EVT_TRIPLO", displayName: "Trip Low Event" },
	{ name: "ADC_EVT_ZERO", displayName: "Zero Crossing Event" },
]
let ADC_FORCE = [
	{ name: "ADC_FORCE_SOC0", displayName: "SW trigger ADC SOC 0" },
	{ name: "ADC_FORCE_SOC1", displayName: "SW trigger ADC SOC 1" },
	{ name: "ADC_FORCE_SOC2", displayName: "SW trigger ADC SOC 2" },
	{ name: "ADC_FORCE_SOC3", displayName: "SW trigger ADC SOC 3" },
	{ name: "ADC_FORCE_SOC4", displayName: "SW trigger ADC SOC 4" },
	{ name: "ADC_FORCE_SOC5", displayName: "SW trigger ADC SOC 5" },
	{ name: "ADC_FORCE_SOC6", displayName: "SW trigger ADC SOC 6" },
	{ name: "ADC_FORCE_SOC7", displayName: "SW trigger ADC SOC 7" },
	{ name: "ADC_FORCE_SOC8", displayName: "SW trigger ADC SOC 8" },
	{ name: "ADC_FORCE_SOC9", displayName: "SW trigger ADC SOC 9" },
	{ name: "ADC_FORCE_SOC10", displayName: "SW trigger ADC SOC 10" },
	{ name: "ADC_FORCE_SOC11", displayName: "SW trigger ADC SOC 11" },
	{ name: "ADC_FORCE_SOC12", displayName: "SW trigger ADC SOC 12" },
	{ name: "ADC_FORCE_SOC13", displayName: "SW trigger ADC SOC 13" },
	{ name: "ADC_FORCE_SOC14", displayName: "SW trigger ADC SOC 14" },
	{ name: "ADC_FORCE_SOC15", displayName: "SW trigger ADC SOC 15" },
]

// let ADC_REPEATER = [
// 	{ name: "ADC_REPEATER_MODULE_BUSY", displayName: "REPEATER MODULE BUSY" },
// 	{ name: "ADC_REPEATER_PHASE_OVERFLOW", displayName: "REPEATER PHASE OVERFLOW" },
// 	{ name: "ADC_REPEATER_TRIGGER_OVERFLOW", displayName: "REPEATER TRIGGER OVERFLOW" },
// ]
// let ADC_REP1CTL = [
// 	{ name: "ADC_REP1CTL_ACTIVEMODE_S", displayName: "REP1CTL ACTIVEMODE S" },
// 	{ name: "ADC_REP1CTL_MODULEBUSY_S", displayName: "REP1CTL MODULEBUSY S" },
// ]
// let ADC_REPSTATUS_MASK = [
// 	{ name: "ADC_REPSTATUS_MASK", displayName: "REPSTATUS MASK" },
// ]
// let ADC_SAFECHECK = [
// 	{ name: "ADC_SAFECHECK_RESULT1_READY", displayName: "SAFECHECK RESULT1 READY" },
// 	{ name: "ADC_SAFECHECK_RESULT2_READY", displayName: "SAFECHECK RESULT2 READY" },
// 	{ name: "ADC_SAFECHECK_RESULT_OOT", displayName: "SAFECHECK RESULT OOT" },
// ]
// let ADC_SAFECHECK_STATUS_MASK = [
// 	{ name: "ADC_SAFECHECK_STATUS_MASK", displayName: "SAFECHECK STATUS MASK" },
// ]

let ADC_SafetyAggr_Instances = [
    {name:  "ADC_SAFETY_CHECKER_AGGR1",
     displayName: "ADC Safety Checker Aggregator 1"},
]

let ADC_Sysclk_Mhz = 200
function getInterfaceName(inst) {

	return "ADC";
}

function getInterfaceNameAdcR(inst) {
    return "ADC_R";
}

function getInterfaceNameAdcSC(inst) {
    return "ADC_SC";
}


const staticConfig = [
    {
        name: "ADC0",
        baseAddr: "CSL_CONTROLSS_ADC0_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADC0_RESULT_U_BASE",
        instanceNumber : "0",
    },
    {
        name: "ADC1",
        baseAddr: "CSL_CONTROLSS_ADC1_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADC1_RESULT_U_BASE",
        instanceNumber : "1",
    },
    {
        name: "ADC2",
        baseAddr: "CSL_CONTROLSS_ADC2_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADC2_RESULT_U_BASE",
        instanceNumber : "2",
    },
    {
        name: "ADC3",
        baseAddr: "CSL_CONTROLSS_ADC3_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADC3_RESULT_U_BASE",
        instanceNumber : "3",
    },
    {
        name: "ADC4",
        baseAddr: "CSL_CONTROLSS_ADC4_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADC4_RESULT_U_BASE",
        instanceNumber : "4",
    },
    {
        name: "ADC_R0",
        baseAddr: "CSL_CONTROLSS_ADCR0_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADCR0_RESULT_U_BASE",
        instanceNumber : "5",
    },
    {
        name: "ADC_R1",
        baseAddr: "CSL_CONTROLSS_ADCR1_U_BASE",
        resultBaseAddr : "CSL_CONTROLSS_ADCR1_RESULT_U_BASE",
        instanceNumber : "6",
    },
    {
        name: "ADC_SAFETY_CHECKER1",
        baseAddr: "CSL_CONTROLSS_ADCSAFE0_U_BASE",
        instanceNumber : "0",
    },
    {
        name: "ADC_SAFETY_CHECKER2",
        baseAddr: "CSL_CONTROLSS_ADCSAFE1_U_BASE",
        instanceNumber : "1",
    },
    {
        name: "ADC_SAFETY_CHECKER3",
        baseAddr: "CSL_CONTROLSS_ADCSAFE2_U_BASE",
        instanceNumber : "2",
    },
    {
        name: "ADC_SAFETY_CHECKER4",
        baseAddr: "CSL_CONTROLSS_ADCSAFE3_U_BASE",
        instanceNumber : "3",
    },
    {
        name: "ADC_SAFETY_CHECKER5",
        baseAddr: "CSL_CONTROLSS_ADCSAFE4_U_BASE",
        instanceNumber : "4",
    },
    {
        name: "ADC_SAFETY_CHECKER6",
        baseAddr: "CSL_CONTROLSS_ADCSAFE5_U_BASE",
        instanceNumber : "5",
    },
    {
        name: "ADC_SAFETY_CHECKER7",
        baseAddr: "CSL_CONTROLSS_ADCSAFE6_U_BASE",
        instanceNumber : "6",
    },
    {
        name: "ADC_SAFETY_CHECKER8",
        baseAddr: "CSL_CONTROLSS_ADCSAFE7_U_BASE",
        instanceNumber : "7",
    },
    {
        name: "ADC_SAFETY_CHECKER9",
        baseAddr: "CSL_CONTROLSS_ADCSAFE8_U_BASE",
        instanceNumber : "8",
    },
    {
        name: "ADC_SAFETY_CHECKER10",
        baseAddr: "CSL_CONTROLSS_ADCSAFE9_U_BASE",
        instanceNumber : "9",
    },
    {
        name: "ADC_SAFETY_CHECKER11",
        baseAddr: "CSL_CONTROLSS_ADCSAFE10_U_BASE",
        instanceNumber : "10",
    },
    {
        name: "ADC_SAFETY_CHECKER12",
        baseAddr: "CSL_CONTROLSS_ADCSAFE11_U_BASE",
        instanceNumber : "11",
    },
    {
        name: "ADC_SAFETY_CHECKER_AGGR1",
        baseAddr: "CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE",
        instanceNumber : "1",
    },
];

const staticConfigAdcR = [
];
function getStaticConfigArr(instance) {
    return staticConfig;
}

exports = {
	ADC_ClkPrescale: ADC_ClkPrescale,
	ADC_Resolution: ADC_Resolution,
	ADC_SignalMode: ADC_SignalMode,
	ADC_Trigger: ADC_Trigger,
	ADC_Channel: ADC_Channel,
	ADC_R_Channel: ADC_R_Channel,
	ADC_PulseMode: ADC_PulseMode,
	ADC_IntNumber: ADC_IntNumber,
	ADC_PPBNumber: ADC_PPBNumber,
	ADC_SOCNumber: ADC_SOCNumber,
	ADC_IntSOCTrigger: ADC_IntSOCTrigger,
	ADC_PriorityMode: ADC_PriorityMode,
	ADC_OSDetectMode: ADC_OSDetectMode,
    ADC_OffsetTrim: ADC_OffsetTrim,
    ADC_SyncInput: ADC_SyncInput,
    ADC_PPBIntSrcSelect: ADC_PPBIntSrcSelect,
    ADC_ExtChannel: ADC_ExtChannel,
    ADC_IntTrigger: ADC_IntTrigger,
    ADC_PPBCompSource: ADC_PPBCompSource,
    ADC_Select: ADC_Select,
    ADC_ResultSelect: ADC_ResultSelect,
    ADC_SafetyCheckerInput: ADC_SafetyCheckerInput,
    ADC_SafetyCheckInst: ADC_SafetyCheckInst,
    ADC_SafetyCheckEvent: ADC_SafetyCheckEvent,
    ADC_SafetyCheckResult: ADC_SafetyCheckResult,
    ADC_Checker: ADC_Checker,
    ADC_SafetyCheckFlag: ADC_SafetyCheckFlag,
    ADC_RepInstance: ADC_RepInstance,
    ADC_RepMode: ADC_RepMode,
	ADC_EVT: ADC_EVT,
	ADC_FORCE: ADC_FORCE,
    ADC_Sysclk_Mhz: ADC_Sysclk_Mhz,
    ADC_SafetyAggr_Instances,
    getInterfaceName,
    getInterfaceNameAdcR,
    getStaticConfigArr,
    getInterfaceNameAdcSC,
}
