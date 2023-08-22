let RESOLVER_excFreqAmplitudeMax = 249 // RDC_MAX_EXCITATION_AMPLITUDE
let RESOLVER_excFreqPhaseMin = 0 // RDC_EXCITATION_FREQUENCY_MIN_PHASE
let RESOLVER_excFreqPhaseMax = 7999 // RDC_EXCITATION_FREQUENCY_MAX_PHASE

let RESOLVER_SequencerMode = [
    {name : "RDC_SEQUENCER_MODE_0", displayName : "RDC_SEQUENCER_MODE_0"},
    {name : "RDC_SEQUENCER_MODE_1", displayName : "RDC_SEQUENCER_MODE_1"},
    {name : "RDC_SEQUENCER_MODE_2", displayName : "RDC_SEQUENCER_MODE_2"},
    {name : "RDC_SEQUENCER_MODE_3", displayName : "RDC_SEQUENCER_MODE_3"},
    {name : "RDC_SEQUENCER_MODE_4", displayName : "RDC_SEQUENCER_MODE_4"},
    {name : "RDC_SEQUENCER_MODE_5", displayName : "RDC_SEQUENCER_MODE_5"},
]

let RESOLVER_SignalMode = [
	{ name: "RDC_SINGALMODE_DIFFERENTIAL_ENDED", displayName: "Differential mode of sampling" },
	{ name: "RDC_SINGALMODE_SINGLE_ENDED", displayName: "Single mode of sampling" },
]

let RESOLVER_overSamplingRatio = [
	// { name: "OVERSAMPLING_RATIO_16", displayName: "Oversample 16 ADC samples" },
	{ name: "OVERSAMPLING_RATIO_20", displayName: "Oversample 20 ADC samples" },
]

let RESOLVER_ExcFreq = [
	{ name: "RDC_EXCITATION_FREQUENCY_5K", displayName: "Excitation Sine Frequency 5 KHz" },
	{ name: "RDC_EXCITATION_FREQUENCY_10K", displayName: "Excitation Sine Frequency 10 KHz" },
	{ name: "RDC_EXCITATION_FREQUENCY_20K", displayName: "Excitation Sine Frequency 20 KHz" },
]

let RESOLVER_AdcBurstCount = [
	{ name: "RDC_ADC_BURST_COUNT_DISABLE",  displayName: "Burst Count is disabled" },
	{ name: "RDC_ADC_BURST_COUNT_2",        displayName: "2 ADC Samples to be averaged" },
	{ name: "RDC_ADC_BURST_COUNT_4",        displayName: "4 ADC Samples to be averaged" },
	{ name: "RDC_ADC_BURST_COUNT_8",        displayName: "8 ADC Samples to be averaged" },
	{ name: "RDC_ADC_BURST_COUNT_16",       displayName: "16 ADC Samples to be averaged" },
	{ name: "RDC_ADC_BURST_COUNT_32",       displayName: "32 ADC Samples to be averaged" },
]

let RESOLVER_InterruptSources = [
    { name: "RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR",            displayName: "Interrupt On Lowamplitude Error" },
    { name: "RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR", displayName: "Interrupt On Highamplitude Cos Fault Error" },
    { name: "RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR", displayName: "Interrupt On Highamplitude Sin Fault Error" },
    { name: "RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR",           displayName: "Interrupt On Sinsqcossq Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR",           displayName: "Interrupt On Sinsqcossq Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_COS_MULTI_ZC_ERROR_ERR",      displayName: "Interrupt On Cos Multi Zc Error Error" },
    { name: "RDC_INTERRUPT_SOURCE_SIN_MULTI_ZC_ERROR_ERR",      displayName: "Interrupt On Sin Multi Zc Error Error" },
    { name: "RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR",displayName: "Interrupt On Cos Neg Zc Peak Mismatch Error" },
    { name: "RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR",displayName: "Interrupt On Cos Pos Zc Peak Mismatch Error" },
    { name: "RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR",displayName: "Interrupt On Sin Neg Zc Peak Mismatch Error" },
    { name: "RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR",displayName: "Interrupt On Sin Pos Zc Peak Mismatch Error" },
    { name: "RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_SIN_LO_ERR",     displayName: "Interrupt On Excfreqdrift Sin Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_COS_LO_ERR",     displayName: "Interrupt On Excfreqdrift Cos Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_EXCFREQDRIFT_HI_ERR",         displayName: "Interrupt On Excfreqdrift Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR",       displayName: "Interrupt On Phasedrift Cos Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR",       displayName: "Interrupt On Phasedrift Cos Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR",        displayName: "Interrupt On Gaindrift Sin Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR",        displayName: "Interrupt On Gaindrift Sin Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR",        displayName: "Interrupt On Gaindrift Cos Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR",        displayName: "Interrupt On Gaindrift Cos Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR",      displayName: "Interrupt On Offsetdrift Sin Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR",      displayName: "Interrupt On Offsetdrift Sin Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR",      displayName: "Interrupt On Offsetdrift Cos Lo Error" },
    { name: "RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR",      displayName: "Interrupt On Offsetdrift Cos Hi Error" },
    { name: "RDC_INTERRUPT_SOURCE_TRACK_LOCK_ERR",              displayName: "Interrupt On Track Lock Error" },
]

let RESOLVER_idealSampleMode    =   [
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT",         displayName: "Ideal Sample Time Mode 0 Auto Detect" },
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN",  displayName: "Ideal Sample Time Mode 1 Auto Detect On Sin" },
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_COS",  displayName: "Ideal Sample Time Mode 1 Auto Detect On Cos" },
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF",     displayName: "Ideal Sample Time Mode 3 Auto Detect Off" },
]
//---------------------------------//
let RESOLVER_Trigger = [
    { name: "RESOLVER_TRIGGER_SW_ONLY", displayName: "Software only" },
    { name: "RESOLVER_TRIGGER_RTI0", displayName: "RTI Timer 0" },
    { name: "RESOLVER_TRIGGER_RTI1", displayName: "RTI Timer 1" },
    { name: "RESOLVER_TRIGGER_RTI2", displayName: "RTI Timer 2" },
    { name: "RESOLVER_TRIGGER_RTI3", displayName: "RTI Timer 3" },
    { name: "RESOLVER_TRIGGER_INPUT_XBAR_OUT5", displayName: "InputXBar.Out[5]" },
    { name: "RESOLVER_TRIGGER_EPWM0_SOCA", displayName: "ePWM0, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM0_SOCB", displayName: "ePWM0, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM1_SOCA", displayName: "ePWM1, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM1_SOCB", displayName: "ePWM1, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM2_SOCA", displayName: "ePWM2, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM2_SOCB", displayName: "ePWM2, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM3_SOCA", displayName: "ePWM3, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM3_SOCB", displayName: "ePWM3, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM4_SOCA", displayName: "ePWM4, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM4_SOCB", displayName: "ePWM4, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM5_SOCA", displayName: "ePWM5, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM5_SOCB", displayName: "ePWM5, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM6_SOCA", displayName: "ePWM6, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM6_SOCB", displayName: "ePWM6, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM7_SOCA", displayName: "ePWM7, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM7_SOCB", displayName: "ePWM7, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM8_SOCA", displayName: "ePWM8, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM8_SOCB", displayName: "ePWM8, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM9_SOCA", displayName: "ePWM9, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM9_SOCB", displayName: "ePWM9, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM10_SOCA", displayName: "ePWM10, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM10_SOCB", displayName: "ePWM10, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM11_SOCA", displayName: "ePWM11, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM11_SOCB", displayName: "ePWM11, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM12_SOCA", displayName: "ePWM12, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM12_SOCB", displayName: "ePWM12, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM13_SOCA", displayName: "ePWM13, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM13_SOCB", displayName: "ePWM13, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM14_SOCA", displayName: "ePWM14, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM14_SOCB", displayName: "ePWM14, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM15_SOCA", displayName: "ePWM15, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM15_SOCB", displayName: "ePWM15, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM16_SOCA", displayName: "ePWM16, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM16_SOCB", displayName: "ePWM16, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM17_SOCA", displayName: "ePWM17, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM17_SOCB", displayName: "ePWM17, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM18_SOCA", displayName: "ePWM18, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM18_SOCB", displayName: "ePWM18, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM19_SOCA", displayName: "ePWM19, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM19_SOCB", displayName: "ePWM19, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM20_SOCA", displayName: "ePWM20, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM20_SOCB", displayName: "ePWM20, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM21_SOCA", displayName: "ePWM21, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM21_SOCB", displayName: "ePWM21, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM22_SOCA", displayName: "ePWM22, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM22_SOCB", displayName: "ePWM22, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM23_SOCA", displayName: "ePWM23, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM23_SOCB", displayName: "ePWM23, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM24_SOCA", displayName: "ePWM24, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM24_SOCB", displayName: "ePWM24, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM25_SOCA", displayName: "ePWM25, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM25_SOCB", displayName: "ePWM25, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM26_SOCA", displayName: "ePWM26, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM26_SOCB", displayName: "ePWM26, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM27_SOCA", displayName: "ePWM27, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM27_SOCB", displayName: "ePWM27, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM28_SOCA", displayName: "ePWM28, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM28_SOCB", displayName: "ePWM28, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM29_SOCA", displayName: "ePWM29, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM29_SOCB", displayName: "ePWM29, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM30_SOCA", displayName: "ePWM30, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM30_SOCB", displayName: "ePWM30, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_EPWM31_SOCA", displayName: "ePWM31, RESOLVERSOCA" },
    { name: "RESOLVER_TRIGGER_EPWM31_SOCB", displayName: "ePWM31, RESOLVERSOCB" },
    { name: "RESOLVER_TRIGGER_ECAP0_SOCEVT", displayName: "eCAP0, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP1_SOCEVT", displayName: "eCAP1, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP2_SOCEVT", displayName: "eCAP2, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP3_SOCEVT", displayName: "eCAP3, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP4_SOCEVT", displayName: "eCAP4, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP5_SOCEVT", displayName: "eCAP5, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP6_SOCEVT", displayName: "eCAP6, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP7_SOCEVT", displayName: "eCAP7, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP8_SOCEVT", displayName: "eCAP8, SOCEVT" },
    { name: "RESOLVER_TRIGGER_ECAP9_SOCEVT", displayName: "eCAP9, SOCEVT" },
]
let RESOLVER_Channel = [
	{ name: "RESOLVER_CH_RESOLVERIN0", displayName: "single-ended, RESOLVERIN0" },
	{ name: "RESOLVER_CH_RESOLVERIN1", displayName: "single-ended, RESOLVERIN1" },
	{ name: "RESOLVER_CH_RESOLVERIN2", displayName: "single-ended, RESOLVERIN2" },
	{ name: "RESOLVER_CH_RESOLVERIN3", displayName: "single-ended, RESOLVERIN3" },
	{ name: "RESOLVER_CH_RESOLVERIN4", displayName: "single-ended, RESOLVERIN4" },
	{ name: "RESOLVER_CH_RESOLVERIN5", displayName: "single-ended, RESOLVERIN5" },
	{ name: "RESOLVER_CH_RESOLVERIN0_RESOLVERIN1", displayName: "differential, RESOLVERIN0 and RESOLVERIN1" },
	{ name: "RESOLVER_CH_RESOLVERIN1_RESOLVERIN0", displayName: "differential, RESOLVERIN1 and RESOLVERIN0" },
	{ name: "RESOLVER_CH_RESOLVERIN2_RESOLVERIN3", displayName: "differential, RESOLVERIN2 and RESOLVERIN3" },
	{ name: "RESOLVER_CH_RESOLVERIN3_RESOLVERIN2", displayName: "differential, RESOLVERIN3 and RESOLVERIN2" },
	{ name: "RESOLVER_CH_RESOLVERIN4_RESOLVERIN5", displayName: "differential, RESOLVERIN4 and RESOLVERIN5" },
    { name: "RESOLVER_CH_RESOLVERIN5_RESOLVERIN4", displayName: "differential, RESOLVERIN5 and RESOLVERIN4" },

]
// let RESOLVER_PulseMode = [
// 	{ name: "RESOLVER_PULSE_END_OF_ACQ_WIN", displayName: "Occurs at the end of the acquisition window" },
// 	{ name: "RESOLVER_PULSE_END_OF_CONV", displayName: "Occurs at the end of the conversion" },
// ]
// let RESOLVER_IntNumber = [
// 	{ name: "RESOLVER_INT_NUMBER1", displayName: "RESOLVERINT1 Interrupt" },
// 	{ name: "RESOLVER_INT_NUMBER2", displayName: "RESOLVERINT2 Interrupt" },
// 	{ name: "RESOLVER_INT_NUMBER3", displayName: "RESOLVERINT3 Interrupt" },
// 	{ name: "RESOLVER_INT_NUMBER4", displayName: "RESOLVERINT4 Interrupt" },
// ]
// let RESOLVER_PPBNumber = [
// 	{ name: "RESOLVER_PPB_NUMBER1", displayName: "Post-processing block 1" },
// 	{ name: "RESOLVER_PPB_NUMBER2", displayName: "Post-processing block 2" },
// 	{ name: "RESOLVER_PPB_NUMBER3", displayName: "Post-processing block 3" },
// 	{ name: "RESOLVER_PPB_NUMBER4", displayName: "Post-processing block 4" },
// ]
// let RESOLVER_SOCNumber = [
// 	{ name: "RESOLVER_SOC_NUMBER0", displayName: "SOC/EOC number 0" },
// 	{ name: "RESOLVER_SOC_NUMBER1", displayName: "SOC/EOC number 1" },
// 	{ name: "RESOLVER_SOC_NUMBER2", displayName: "SOC/EOC number 2" },
// 	{ name: "RESOLVER_SOC_NUMBER3", displayName: "SOC/EOC number 3" },
// 	{ name: "RESOLVER_SOC_NUMBER4", displayName: "SOC/EOC number 4" },
// 	{ name: "RESOLVER_SOC_NUMBER5", displayName: "SOC/EOC number 5" },
// 	{ name: "RESOLVER_SOC_NUMBER6", displayName: "SOC/EOC number 6" },
// 	{ name: "RESOLVER_SOC_NUMBER7", displayName: "SOC/EOC number 7" },
// 	{ name: "RESOLVER_SOC_NUMBER8", displayName: "SOC/EOC number 8" },
// 	{ name: "RESOLVER_SOC_NUMBER9", displayName: "SOC/EOC number 9" },
// 	{ name: "RESOLVER_SOC_NUMBER10", displayName: "SOC/EOC number 10" },
// 	{ name: "RESOLVER_SOC_NUMBER11", displayName: "SOC/EOC number 11" },
// 	{ name: "RESOLVER_SOC_NUMBER12", displayName: "SOC/EOC number 12" },
// 	{ name: "RESOLVER_SOC_NUMBER13", displayName: "SOC/EOC number 13" },
// 	{ name: "RESOLVER_SOC_NUMBER14", displayName: "SOC/EOC number 14" },
// 	{ name: "RESOLVER_SOC_NUMBER15", displayName: "SOC/EOC number 15" },
// ]
// let RESOLVER_IntSOCTrigger = [
// 	{ name: "RESOLVER_INT_SOC_TRIGGER_NONE", displayName: "No RESOLVERINT will trigger the SOC" },
// 	{ name: "RESOLVER_INT_SOC_TRIGGER_RESOLVERINT1", displayName: "RESOLVERINT1 will trigger the SOC" },
// 	{ name: "RESOLVER_INT_SOC_TRIGGER_RESOLVERINT2", displayName: "RESOLVERINT2 will trigger the SOC" },
// ]
// let RESOLVER_PriorityMode = [
// 	{ name: "RESOLVER_PRI_ALL_ROUND_ROBIN", displayName: "Round robin mode is used for all" },
// 	{ name: "RESOLVER_PRI_SOC0_HIPRI", displayName: "SOC 0 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC1_HIPRI", displayName: "SOC 0-1 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC2_HIPRI", displayName: "SOC 0-2 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC3_HIPRI", displayName: "SOC 0-3 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC4_HIPRI", displayName: "SOC 0-4 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC5_HIPRI", displayName: "SOC 0-5 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC6_HIPRI", displayName: "SOC 0-6 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC7_HIPRI", displayName: "SOC 0-7 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC8_HIPRI", displayName: "SOC 0-8 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC9_HIPRI", displayName: "SOC 0-9 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC10_HIPRI", displayName: "SOC 0-10 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC11_HIPRI", displayName: "SOC 0-11 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC12_HIPRI", displayName: "SOC 0-12 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC13_HIPRI", displayName: "SOC 0-13 hi pri, others in round robin" },
// 	{ name: "RESOLVER_PRI_THRU_SOC14_HIPRI", displayName: "SOC 0-14 hi pri, SOC15 in round robin" },
// 	{ name: "RESOLVER_PRI_ALL_HIPRI", displayName: "All priorities based on SOC number" },
// ]
// let RESOLVER_OSDetectMode = [
// 	{ name: "RESOLVER_OSDETECT_MODE_DISABLED", displayName: "Open/Shorts detection cir-" },
// 	{ name: "RESOLVER_OSDETECT_MODE_VSSA", displayName: "O/S DC is enabled at zero" },
// 	{ name: "RESOLVER_OSDETECT_MODE_VDDA", displayName: "O/S DC is enabled at full" },
// 	{ name: "RESOLVER_OSDETECT_MODE_5BY12_VDDA", displayName: "O/S DC is enabled at 5/12" },
// 	{ name: "RESOLVER_OSDETECT_MODE_7BY12_VDDA", displayName: "O/S DC is enabled at 7/12" },
// 	{ name: "RESOLVER_OSDETECT_MODE_5K_PULLDOWN_TO_VSSA", displayName: "O/S DC is enabled at 5K" },
// 	{ name: "RESOLVER_OSDETECT_MODE_5K_PULLUP_TO_VDDA", displayName: "O/S DC is enabled at 5K" },
// 	{ name: "RESOLVER_OSDETECT_MODE_7K_PULLDOWN_TO_VSSA", displayName: "O/S DC is enabled at 7K" },
// ]
// let RESOLVER_EVT = [
// 	{ name: "RESOLVER_EVT_TRIPHI", displayName: "Trip High Event" },
// 	{ name: "RESOLVER_EVT_TRIPLO", displayName: "Trip Low Event" },
// 	{ name: "RESOLVER_EVT_ZERO", displayName: "Zero Crossing Event" },
// ]
// let RESOLVER_FORCE = [
// 	{ name: "RESOLVER_FORCE_SOC0", displayName: "SW trigger RESOLVER SOC 0" },
// 	{ name: "RESOLVER_FORCE_SOC1", displayName: "SW trigger RESOLVER SOC 1" },
// 	{ name: "RESOLVER_FORCE_SOC2", displayName: "SW trigger RESOLVER SOC 2" },
// 	{ name: "RESOLVER_FORCE_SOC3", displayName: "SW trigger RESOLVER SOC 3" },
// 	{ name: "RESOLVER_FORCE_SOC4", displayName: "SW trigger RESOLVER SOC 4" },
// 	{ name: "RESOLVER_FORCE_SOC5", displayName: "SW trigger RESOLVER SOC 5" },
// 	{ name: "RESOLVER_FORCE_SOC6", displayName: "SW trigger RESOLVER SOC 6" },
// 	{ name: "RESOLVER_FORCE_SOC7", displayName: "SW trigger RESOLVER SOC 7" },
// 	{ name: "RESOLVER_FORCE_SOC8", displayName: "SW trigger RESOLVER SOC 8" },
// 	{ name: "RESOLVER_FORCE_SOC9", displayName: "SW trigger RESOLVER SOC 9" },
// 	{ name: "RESOLVER_FORCE_SOC10", displayName: "SW trigger RESOLVER SOC 10" },
// 	{ name: "RESOLVER_FORCE_SOC11", displayName: "SW trigger RESOLVER SOC 11" },
// 	{ name: "RESOLVER_FORCE_SOC12", displayName: "SW trigger RESOLVER SOC 12" },
// 	{ name: "RESOLVER_FORCE_SOC13", displayName: "SW trigger RESOLVER SOC 13" },
// 	{ name: "RESOLVER_FORCE_SOC14", displayName: "SW trigger RESOLVER SOC 14" },
// 	{ name: "RESOLVER_FORCE_SOC15", displayName: "SW trigger RESOLVER SOC 15" },
// ]
let RESOLVER_Sysclk_Mhz = 200
function getInterfaceName(instance) {
    return "RESOLVER";
}
const staticConfig = [
    {
        name: "RESOLVER",
        baseAddr: "CSL_CONTROLSS_HW_RESOLVER_U_BASE",
    },
];
function getStaticConfigArr() {
    return staticConfig;
}
exports = {
    RESOLVER_excFreqAmplitudeMax,
    RESOLVER_excFreqPhaseMin,
    RESOLVER_excFreqPhaseMax,
    RESOLVER_SequencerMode,
    RESOLVER_InterruptSources,
    RESOLVER_idealSampleMode,
	// RESOLVER_ClkPrescale: RESOLVER_ClkPrescale,
	// RESOLVER_Resolution: RESOLVER_Resolution,
    RESOLVER_AdcBurstCount,
    RESOLVER_overSamplingRatio,
	RESOLVER_SignalMode: RESOLVER_SignalMode,
    RESOLVER_ExcFreq,
	RESOLVER_Trigger: RESOLVER_Trigger,
	RESOLVER_Channel: RESOLVER_Channel,
	// RESOLVER_PulseMode: RESOLVER_PulseMode,
	// RESOLVER_IntNumber: RESOLVER_IntNumber,
	// RESOLVER_PPBNumber: RESOLVER_PPBNumber,
	// RESOLVER_SOCNumber: RESOLVER_SOCNumber,
	// RESOLVER_IntSOCTrigger: RESOLVER_IntSOCTrigger,
	// RESOLVER_PriorityMode: RESOLVER_PriorityMode,
	// RESOLVER_OSDetectMode: RESOLVER_OSDetectMode,
	// RESOLVER_EVT: RESOLVER_EVT,
	// RESOLVER_FORCE: RESOLVER_FORCE,
    RESOLVER_Sysclk_Mhz: RESOLVER_Sysclk_Mhz,
    getInterfaceName,
    getStaticConfigArr,
}
