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
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_2_AUTO_DETECT_ON_COS",  displayName: "Ideal Sample Time Mode 1 Auto Detect On Cos" },
    { name: "RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF",     displayName: "Ideal Sample Time Mode 3 Auto Detect Off" },
]


let RESOLVER_pgEstimationTrainLimitOptions = [
	{ name: 0, displayName: (Math.pow(2,0)).toString()+" Rotation" },
	{ name: 1, displayName: (Math.pow(2,1)).toString()+" Rotations" },
	{ name: 2, displayName: (Math.pow(2,2)).toString()+" Rotations" },
	{ name: 3, displayName: (Math.pow(2,3)).toString()+" Rotations" },
	{ name: 4, displayName: (Math.pow(2,4)).toString()+" Rotations" },
	{ name: 5, displayName: (Math.pow(2,5)).toString()+" Rotations" },
	{ name: 6, displayName: (Math.pow(2,6)).toString()+" Rotations" },
	{ name: 7, displayName: (Math.pow(2,7)).toString()+" Rotations" },
	{ name: 8, displayName: (Math.pow(2,8)).toString()+" Rotations" },
	{ name: 9, displayName: (Math.pow(2,9)).toString()+" Rotations" },
	{ name: 10, displayName: (Math.pow(2,10)).toString()+" Rotations" },
	{ name: 11, displayName: (Math.pow(2,11)).toString()+" Rotations" },
	{ name: 12, displayName: (Math.pow(2,12)).toString()+" Rotations" },
	{ name: 13, displayName: (Math.pow(2,13)).toString()+" Rotations" },
	{ name: 14, displayName: (Math.pow(2,14)).toString()+" Rotations" },
	{ name: 15, displayName: (Math.pow(2,15)).toString()+" Rotations" },
]

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
    RESOLVER_AdcBurstCount,
    RESOLVER_overSamplingRatio,
	RESOLVER_SignalMode: RESOLVER_SignalMode,
    RESOLVER_ExcFreq,
    RESOLVER_pgEstimationTrainLimitOptions,
    RESOLVER_Sysclk_Mhz: RESOLVER_Sysclk_Mhz,
    getInterfaceName,
    getStaticConfigArr,
}
