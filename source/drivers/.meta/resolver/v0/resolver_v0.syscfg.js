let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript("/drivers/resolver/soc/resolver_am263px.syscfg.js");
let resolver_sampletime_sysclk_ns = 1000/device_peripheral.RESOLVER_Sysclk_Mhz;

function getStaticConfigArr() {
    return system.getScript(`/drivers/resolver/soc/resolver_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
}

let config = [
    {
        name        :   "advConfig",
        displayName :   "enable Advanced configurations",
        description :   "Check this box to enable configurations on the advanced parameters",
        default     :   false,
        onChange    :   (inst, ui) => {
            if(inst["advConfig"] == true)
            {
                ui["resolverExcitationFrequencyAmplitudeAdv"].readOnly = false
                ui["socDelayAdv"].readOnly = false

                ui["core0dcOffsetCalCoef1Adv"].readOnly = false
                ui["core1dcOffsetCalCoef1Adv"].readOnly = false
                ui["core0dcOffsetCalCoef2Adv"].readOnly = false
                ui["core1dcOffsetCalCoef2Adv"].readOnly = false

                ui["core0IdealSampleOverrideValueAdv"].readOnly = false
                ui["core1IdealSampleOverrideValueAdv"].readOnly = false
                ui["core0IdealSampleDetectionThresholdAdv"].readOnly = false
                ui["core1IdealSampleDetectionThresholdAdv"].readOnly = false
                ui["core0IdealSampleBpfAdjustAdv"].readOnly = false
                ui["core1IdealSampleBpfAdjustAdv"].readOnly = false
                // ui["core0IdealSamplePeakAvgLimitAdv"].readOnly = false
                // ui["core1IdealSamplePeakAvgLimitAdv"].readOnly = false
                ui["core0IdealSampleModeAdv"].readOnly = false
                ui["core1IdealSampleModeAdv"].readOnly = false
                ui["core0IdealSampleBottomSamplingAdv"].readOnly = false
                ui["core1IdealSampleBottomSamplingAdv"].readOnly = false

                // ui["core0PgOffsetConfigAdv"].readOnly = false
                // ui["core1PgOffsetConfigAdv"].readOnly = false
                ui["core0PgEstimationEnableAdv"].readOnly = false
                ui["core1PgEstimationEnableAdv"].readOnly = false
                // ui["core0PgGlitchThresholdAdv"].readOnly = false
                // ui["core1PgGlitchThresholdAdv"].readOnly = false
                ui["core0PgEstimationTrainLimitAdv"].readOnly = false
                ui["core1PgEstimationTrainLimitAdv"].readOnly = false
                ui["core0PgCorrectionEnableAdv"].readOnly = false
                ui["core1PgCorrectionEnableAdv"].readOnly = false
                ui["core0PgAutoPhaseGainCorrectionAdv"].readOnly = false
                ui["core1PgAutoPhaseGainCorrectionAdv"].readOnly = false
                ui["core0PgSinGainBypassValueAdv"].readOnly = false
                ui["core1PgSinGainBypassValueAdv"].readOnly = false
                ui["core0PgCosGainBypassValueAdv"].readOnly = false
                ui["core1PgCosGainBypassValueAdv"].readOnly = false
                ui["core0PgCosPhaseBypassValueAdv"].readOnly = false
                ui["core1PgCosPhaseBypassValueAdv"].readOnly = false

                // ui["core0track2kiAdv"].readOnly = false
                // ui["core1track2kiAdv"].readOnly = false
                // ui["core0track2kffwAdv"].readOnly = false
                // ui["core1track2kffwAdv"].readOnly = false
                // ui["core0track2kpdivAdv"].readOnly = false
                // ui["core1track2kpdivAdv"].readOnly = false
                ui["core0track2kvelfiltAdv"].readOnly = false
                ui["core1track2kvelfiltAdv"].readOnly = false
                // ui["core0track2vboostcoefAdv"].readOnly = false
                // ui["core1track2vboostcoefAdv"].readOnly = false

            }
            else
            {
                ui["resolverExcitationFrequencyAmplitudeAdv"].readOnly = true
                ui["socDelayAdv"].readOnly = true

                ui["core0dcOffsetCalCoef1Adv"].readOnly = true
                ui["core1dcOffsetCalCoef1Adv"].readOnly = true
                ui["core0dcOffsetCalCoef2Adv"].readOnly = true
                ui["core1dcOffsetCalCoef2Adv"].readOnly = true

                ui["core0IdealSampleOverrideValueAdv"].readOnly = true
                ui["core1IdealSampleOverrideValueAdv"].readOnly = true
                ui["core0IdealSampleDetectionThresholdAdv"].readOnly = true
                ui["core1IdealSampleDetectionThresholdAdv"].readOnly = true
                ui["core0IdealSampleBpfAdjustAdv"].readOnly = true
                ui["core1IdealSampleBpfAdjustAdv"].readOnly = true
                ui["core0IdealSamplePeakAvgLimitAdv"].readOnly = true
                ui["core1IdealSamplePeakAvgLimitAdv"].readOnly = true
                ui["core0IdealSampleModeAdv"].readOnly = true
                ui["core1IdealSampleModeAdv"].readOnly = true
                ui["core0IdealSampleBottomSamplingAdv"].readOnly = true
                ui["core1IdealSampleBottomSamplingAdv"].readOnly = true

                // ui["core0PgOffsetConfigAdv"].readOnly = true
                // ui["core1PgOffsetConfigAdv"].readOnly = true
                ui["core0PgEstimationEnableAdv"].readOnly = true
                ui["core1PgEstimationEnableAdv"].readOnly = true
                // ui["core0PgGlitchThresholdAdv"].readOnly = true
                // ui["core1PgGlitchThresholdAdv"].readOnly = true
                ui["core0PgEstimationTrainLimitAdv"].readOnly = true
                ui["core1PgEstimationTrainLimitAdv"].readOnly = true
                ui["core0PgCorrectionEnableAdv"].readOnly = true
                ui["core1PgCorrectionEnableAdv"].readOnly = true
                ui["core0PgAutoPhaseGainCorrectionAdv"].readOnly = true
                ui["core1PgAutoPhaseGainCorrectionAdv"].readOnly = true
                ui["core0PgSinGainBypassValueAdv"].readOnly = true
                ui["core1PgSinGainBypassValueAdv"].readOnly = true
                ui["core0PgCosGainBypassValueAdv"].readOnly = true
                ui["core1PgCosGainBypassValueAdv"].readOnly = true
                ui["core0PgCosPhaseBypassValueAdv"].readOnly = true
                ui["core1PgCosPhaseBypassValueAdv"].readOnly = true

                ui["core0track2kiAdv"].readOnly = true
                ui["core1track2kiAdv"].readOnly = true
                ui["core0track2kffwAdv"].readOnly = true
                ui["core1track2kffwAdv"].readOnly = true
                ui["core0track2kpdivAdv"].readOnly = true
                ui["core1track2kpdivAdv"].readOnly = true
                ui["core0track2kvelfiltAdv"].readOnly = true
                ui["core1track2kvelfiltAdv"].readOnly = true
                ui["core0track2vboostcoefAdv"].readOnly = true
                ui["core1track2vboostcoefAdv"].readOnly = true

            }
        }
    }
];

config = config.concat([
    {
        name        : "InputConfigurations",
        displayName : "Input Configurations",
        config      : [
            {
                name        : "resolverSignalMode",
                displayName : "RESOLVER Signal Mode",
                description : 'Select the RESOLVER signal mode.',
                hidden      : false,
                default     : device_peripheral.RESOLVER_SignalMode[0].name,
                options     : device_peripheral.RESOLVER_SignalMode,
            },
            {
                name            : "adcSocWidth",
                displayName     : "ADC SoC Width",
                description :   "Start of Conversion Width for the ADC conversions",
                default         : 16,
                hidden          : false,
            },
            {
                name            : "adcBurstCount",
                displayName     : "ADC Burst Count",
                description     : "Number of Burst Samples to be Averaged per RDC input. This is only valid for Sequencer mode 0",
                longDescription : "The Oversample Ratio is ran on the Averaged Burst Samples. This is only valid for Sequencer mode 0",
                default         : device_peripheral.RESOLVER_AdcBurstCount[0].name,
                options         : device_peripheral.RESOLVER_AdcBurstCount,
                hidden          : false,
            },
            {
                name        : "resolverSequencerMode",
                displayName : "RESOLVER Sequencer Mode",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![](../source/drivers/.meta/resolver/images/Resolver_Sequencer_Modes.png)`,
                hidden      : false,
                default     : device_peripheral.RESOLVER_SequencerMode[0].name,
                options     : device_peripheral.RESOLVER_SequencerMode,
                onChange    : function (inst, ui){
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[0].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = false
                        ui["resolverSequencerModePicture1"].hidden = true
                        ui["resolverSequencerModePicture2"].hidden = true
                        ui["resolverSequencerModePicture3"].hidden = true
                        ui["resolverSequencerModePicture4"].hidden = true
                        ui["resolverSequencerModePicture5"].hidden = true
                        ui["adcBurstCount"].readOnly = false
                        inst["adcBurstCount"] = device_peripheral.RESOLVER_AdcBurstCount[0].name;
                    }
                    else
                    {
                        ui["adcBurstCount"].readOnly = true
                        inst["adcBurstCount"] = device_peripheral.RESOLVER_AdcBurstCount[0].name;
                    }
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[1].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = true
                        ui["resolverSequencerModePicture1"].hidden = false
                        ui["resolverSequencerModePicture2"].hidden = true
                        ui["resolverSequencerModePicture3"].hidden = true
                        ui["resolverSequencerModePicture4"].hidden = true
                        ui["resolverSequencerModePicture5"].hidden = true
                    }
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[2].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = true
                        ui["resolverSequencerModePicture1"].hidden = true
                        ui["resolverSequencerModePicture2"].hidden = false
                        ui["resolverSequencerModePicture3"].hidden = true
                        ui["resolverSequencerModePicture4"].hidden = true
                        ui["resolverSequencerModePicture5"].hidden = true
                    }
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[3].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = true
                        ui["resolverSequencerModePicture1"].hidden = true
                        ui["resolverSequencerModePicture2"].hidden = true
                        ui["resolverSequencerModePicture3"].hidden = false
                        ui["resolverSequencerModePicture4"].hidden = true
                        ui["resolverSequencerModePicture5"].hidden = true
                    }
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[4].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = true
                        ui["resolverSequencerModePicture1"].hidden = true
                        ui["resolverSequencerModePicture2"].hidden = true
                        ui["resolverSequencerModePicture3"].hidden = true
                        ui["resolverSequencerModePicture4"].hidden = false
                        ui["resolverSequencerModePicture5"].hidden = true
                    }
                    if(inst["resolverSequencerMode"] == device_peripheral.RESOLVER_SequencerMode[5].name)
                    {
                        ui["resolverSequencerModePicture0"].hidden = true
                        ui["resolverSequencerModePicture1"].hidden = true
                        ui["resolverSequencerModePicture2"].hidden = true
                        ui["resolverSequencerModePicture3"].hidden = true
                        ui["resolverSequencerModePicture4"].hidden = true
                        ui["resolverSequencerModePicture5"].hidden = false
                    }
                },
            },
            {
                name        : "resolverSequencerModePicture0",
                displayName : "RESOLVER Sequencer Mode 0",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode0_Resolver_Sequencer_Modes.png)`,
                hidden      : false,
                default     : "mode 0 selected. view [?] to see configuration",
                readOnly    : true,
            },
            {
                name        : "resolverSequencerModePicture1",
                displayName : "RESOLVER Sequencer Mode 1",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode1_Resolver_Sequencer_Modes.png)`,
                hidden      : true,
                default     : "mode 1 selected. view [?] to see configuration",
                readOnly    : true,
            },
            {
                name        : "resolverSequencerModePicture2",
                displayName : "RESOLVER Sequencer Mode 2",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode2_Resolver_Sequencer_Modes.png)`,
                hidden      : true,
                default     : "mode 2 selected. view [?] to see configuration",
                readOnly    : true,
            },
            {
                name        : "resolverSequencerModePicture3",
                displayName : "RESOLVER Sequencer Mode 3",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode3_Resolver_Sequencer_Modes.png)`,
                hidden      : true,
                default     : "mode 3 selected. view [?] to see configuration",
                readOnly    : true,
            },
            {
                name        : "resolverSequencerModePicture4",
                displayName : "RESOLVER Sequencer Mode 4",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode4_Resolver_Sequencer_Modes.png)`,
                hidden      : true,
                default     : "mode 4 selected. view [?] to see configuration",
                readOnly    : true,
            },
            {
                name        : "resolverSequencerModePicture5",
                displayName : "RESOLVER Sequencer Mode 5",
                description : 'Sequencer Mode for RESOLVER',
                longDescription : `![size='50%'](../source/drivers/.meta/resolver/images/mode5_Resolver_Sequencer_Modes.png)`,
                hidden      : true,
                default     : "mode 5 selected. view [?] to see configuration",
                readOnly    : true,
            },
        ]
    },
    {
        name        : "Excitation Frequency Configurations",
        displayName : "ExcitationFrequencyConfigurations",
        config      : [
            {
                name        : "resolverExcitationFrequencySelect",
                displayName : "Excitation Frequency Select",
                description : 'Select the Excitation Frequency.',
                hidden      : false,
                default     : device_peripheral.RESOLVER_ExcFreq[2].name,
                options     : device_peripheral.RESOLVER_ExcFreq,
            },
            {
                name        : "resolverExcitationFrequencyPhase",
                displayName : "Excitation Frequency Phase",
                description : 'Select the Excitation Signal Phase.',
                hidden      : false,
                default     : 0,
            },
            {
                name        : "resolverExcitationFrequencyAmplitudeAdv",
                displayName : "Excitation Frequency Amplitude",
                description : 'Select the Excitation Signal Amplitude. To enable editing, please select the advanced configurations above',
                readOnly    : true,
                hidden      : false,
                default     : 249,
            },
            {
                name        : "syncInEnable",
                displayName : "Enable Sync IN",
                // FIXME: Find the PWMxBar that syncs to the resolver from integration spec
                description : 'enables Sync In from the PWMxBar instance 2.',
                hidden      : false,
                default     : false,
            },
            {
                name        : "socDelayAdv",
                displayName : "ADC SOC start Delay",
                description : 'Delay of SOC start from the Excitation Frequency start. To enable editing, please select the advanced configurations above',
                hidden      : false,
                readOnly    : true,
                default     : 0,
            },
            {
                name        : "OverSamplingRatio",
                displayName : "ADC Over Sampling Ratio",
                description : 'Select the ADC Over Sampling Ratio.',
                hidden      : false,
                default     : device_peripheral.RESOLVER_overSamplingRatio[0].name,
                options     : device_peripheral.RESOLVER_overSamplingRatio,
            },
        ]
    },
    // RESOLVER_setPrescaler()

]);

// RESOLVER_setMode()


config = config.concat([
    {
        name        :   "TuningParameters",
        displayName :   "Tuning Parameters configuration",
        description :   "",
        config      :   [
            {
                name        : "manual",
                displayName : "Manual Tuning Parameters",
                hidden      : false,
                default     : true,
                onChange    : (inst, ui) => {
                    if(inst["manual"] == true)
                    {
                        ui["externalFlashLocation_enable"].hidden = false
                        ui["manualPhaseOffset"].hidden = false
                        ui["manualSinGainOffset"].hidden = false
                        ui["manualCosGainOffset"].hidden = false
                        ui["idealSampleOverride"].hidden = false
                    }
                    else if ((inst["manual"] == false))
                    {
                        ui["externalFlashLocation_enable"].hidden = true
                        ui["manualPhaseOffset"].hidden = true
                        ui["manualSinGainOffset"].hidden = true
                        ui["manualCosGainOffset"].hidden = true
                        ui["idealSampleOverride"].hidden = true
                    }
                }
            },
            {
                name        : "externalFlashLocation_enable",
                displayName : "External Flash Location",
                hidden      : false,
                default     : false,
                onChange    : (inst, ui) => {
                    if(inst["externalFlashLocation_enable"] == true)
                    {
                        ui["externalFlashLocation"].hidden = false
                        ui["manualPhaseOffset"].hidden = true
                        ui["manualSinGainOffset"].hidden = true
                        ui["manualCosGainOffset"].hidden = true
                        ui["idealSampleOverride"].hidden = true
                    }
                    else
                    {
                        ui["externalFlashLocation"].hidden = true
                        ui["manualPhaseOffset"].hidden = false
                        ui["manualSinGainOffset"].hidden = false
                        ui["manualCosGainOffset"].hidden = false
                        ui["idealSampleOverride"].hidden = false
                    }
                },
            },
            {
                name        : "externalFlashLocation",
                displayName : "External Flash Memory Location",
                description : "Enter the External Flash Memory location for the tuning Parameters",
                hidden      : true,
                default     : 0,
            },
            {
                name        : "manualPhaseOffset",
                displayName : "Manual Phase Offset Correction Value",
                default     :  0,
                hidden      : false,
                onChange    : (inst, ui) => {
                    inst["core0PgCosPhaseBypassValueAdv"] =  inst["manualPhaseOffset"]
                    inst["core1PgCosPhaseBypassValueAdv"] =  inst["manualPhaseOffset"]
                }
            },
            {
                name        : "manualSinGainOffset",
                displayName : "Manual Sin Gain Correction Value",
                default     :  16384,
                hidden      : false,
                onChange    : (inst, ui) => {
                    inst["core0PgSinGainBypassValueAdv"] =  inst["manualSinGainOffset"]
                    inst["core1PgSinGainBypassValueAdv"] =  inst["manualSinGainOffset"]
                }
            },
            {
                name        : "manualCosGainOffset",
                displayName : "Manual Cos Gain Offset Value",
                default     :  16384,
                hidden      : false,
                onChange    : (inst, ui) => {
                    inst["core0PgCosGainBypassValueAdv"] =  inst["manualCosGainOffset"]
                    inst["core1PgCosGainBypassValueAdv"] =  inst["manualCosGainOffset"]
                }
            },
            {
                name        : "idealSampleOverride",
                displayName : "Manual Ideal Sample Override Value",
                longDescription :   "if the Manual Ideal Sample Override value is set, that value will be reflected in the Ideal Sample Configurations and vice versa",
                default     :  0,
                hidden      : false,
                onChange    : (inst, ui) => {
                    inst["core0IdealSampleOverrideValueAdv"] =  inst["idealSampleOverride"]
                    inst["core1IdealSampleOverrideValueAdv"] =  inst["idealSampleOverride"]
                }
            }
        ],
    },
])


let resovlerCoreConfig = []
for( let core = 0; core <= 1; core++)
{
    let coreConfig = []
    coreConfig = coreConfig.concat([
        {
            name                :   "core"+core.toString()+"bandPassFilterAndDcOffsetConfig",
            displayName         :   "Band Pass Filter and DC Offset Configurations",
            description         :   "",
            longDescription     :   "Band Pass Filter for input ADC OverSample Ratio of 20 is available if Band Pass Filter is Used, DC offset Correction may be ignored",
            config              :   [
                {
                    name        :   "core"+core.toString()+"bandPassFilterEnable",
                    displayName :   "Band Pass Filter Enable",
                    default     :   false,
                    hidden      :   false,
                    onChange    :   (inst, ui) => {
                        if(inst["core"+core.toString()+"bandPassFilterEnable"])
                        {
                            ui["core"+core.toString()+"dcOffsetCorrectionEnable"].readOnly = true
                            // ui["core"+core.toString()+"autoDcOffset"].readOnly = true
                            ui["core"+core.toString()+"sinDcOffset"].readOnly = true
                            ui["core"+core.toString()+"cosDcOffset"].readOnly = true
                        }
                        else
                        {
                            ui["core"+core.toString()+"dcOffsetCorrectionEnable"].readOnly = false
                            // ui["core"+core.toString()+"autoDcOffset"].readOnly = false
                            ui["core"+core.toString()+"sinDcOffset"].readOnly = false
                            ui["core"+core.toString()+"cosDcOffset"].readOnly = false
                        }
                    }
                },
                {
                    name        :   "core"+core.toString()+"DcOffsetConfigurations",
                    displayName :   "DC Offset Configurations",
                    config      :   [
                        {
                            name        :   "core"+core.toString()+"dcOffsetCorrectionEnable",
                            displayName :   "DC Offset Correction Enable",
                            hidden      :   false,
                            default     :   false,
                            readOnly    :   false,
                            onChange    :   (inst, ui) => {
                                if (inst["core"+core.toString()+"dcOffsetCorrectionEnable"] == false)
                                {
                                    // ui["core"+core.toString()+"autoDcOffset"].hidden = true
                                    ui["core"+core.toString()+"sinDcOffset"].hidden = true
                                    ui["core"+core.toString()+"cosDcOffset"].hidden = true
                                    ui["core"+core.toString()+"dcOffsetCalCoef1Adv"].hidden = true
                                    ui["core"+core.toString()+"dcOffsetCalCoef2Adv"].hidden = true
                                }
                                else
                                {
                                    // ui["core"+core.toString()+"autoDcOffset"].hidden = false
                                    ui["core"+core.toString()+"sinDcOffset"].hidden = false
                                    ui["core"+core.toString()+"cosDcOffset"].hidden = false
                                    ui["core"+core.toString()+"sinDcOffset"].readOnly = false
                                    ui["core"+core.toString()+"cosDcOffset"].readOnly = false
                                    ui["core"+core.toString()+"dcOffsetCalCoef1Adv"].hidden = false
                                    ui["core"+core.toString()+"dcOffsetCalCoef2Adv"].hidden = false
                                }
                            }
                        },
                        {
                            name        : "core"+core.toString()+"dcOffsetCalCoef1Adv",
                            displayName : "DC Offset Cal Coefficient 1",
                            hidden      : true,
                            readOnly    : true,
                            default     : "8",
                        },
                        {
                            name        : "core"+core.toString()+"dcOffsetCalCoef2Adv",
                            displayName : "DC Offset Cal Coefficient 2",
                            hidden      : true,
                            readOnly    : true,
                            default     : "9",
                        },
                        // {
                        //     name        :   "core"+core.toString()+"autoDcOffset",
                        //     displayName :   "Auto DC Offset Correction",
                        //     hidden      :   true,
                        //     default     :   false,
                        //     readOnly    :   false,
                        //     onChange    :   (inst, ui) => {
                        //         if(inst["core"+core.toString()+"autoDcOffset"] == true)
                        //         {
                        //             ui["core"+core.toString()+"sinDcOffset"].hidden = true
                        //             ui["core"+core.toString()+"cosDcOffset"].hidden = true
                        //         }
                        //         else
                        //         {
                        //             ui["core"+core.toString()+"sinDcOffset"].hidden = false
                        //             ui["core"+core.toString()+"cosDcOffset"].hidden = false
                        //         }
                        //     }
                        // },
                        {
                            name        :   "core"+core.toString()+"sinDcOffset",
                            displayName :   "Manual Sin DC Offset value",
                            hidden      :   true,
                            readOnly    :   true,
                            default     :   0,
                        },
                        {
                            name        :   "core"+core.toString()+"cosDcOffset",
                            displayName :   "Manual Cos DC Offset value",
                            hidden      :   true,
                            readOnly    :   true,
                            default     :   0,
                        },
                    ],
                },
            ],
        }
    ]);

    coreConfig = coreConfig.concat([
        {
            name            :   "core"+core.toString()+"IdealSampleConfigurations",
            displayName     :   "Ideal Sample Configurations",
            description     :   "",
            longDescription :   "Ideal Sample Configurations. Enable Advanced configurations for editing these values.",
            config          :   [
                {
                    name            :   "core"+core.toString()+"IdealSampleOverrideValueAdv",
                    displayName     :   "Ideal Sample Override Value",
                    description     :   "enable Advanced Configurations to edit this feild",
                    longDescription :   "if the Manual Ideal Sample Override value from the Tuning Parameters is set, that value will be copied here and vice versa",
                    default         :   0,
                    readOnly        :   true,
                    onChange        :   (inst, ui) => {
                        inst["core"+core.toString()+"idealSampleOverride"] =  inst["core"+core.toString()+"IdealSampleOverrideValueAdv"]
                    }
                },
                {
                    name            :   "core"+core.toString()+"IdealSamplePeakAvgLimitAdv",
                    displayName     :   "Peak Averaging Limit",
                    description     :   "enable Advanced Configurations to edit this feild",
                    longDescription :   `
    This is used for the Ideal Sample detection algorithm. Minimum Number of Peaks to be detected over any sample to check the Peak Histogram Buckets and find the Ideal Sample Time to be overridden in the manual mode.
                    `,
                    default         :   0, //FIXME: Update with the reset value
                    readOnly        :   true,
                    hidden          :   true,
                },
                {
                    name            :   "core"+core.toString()+"IdealSampleDetectionThresholdAdv",
                    displayName     :   "Ideal Sample Detection Threshold",
                    longDescription :   "Minimum threshold for the Sample Detection in the Ideal Sample detection Algorithm",
                    default         :   2500,
                    readOnly        :   true,
                },
                {
                    name            :   "core"+core.toString()+"IdealSampleBpfAdjustAdv",
                    displayName     :   "Ideal Sample BPF Adjust",
                    description     :   "enable Advanced Configurations to edit this feild",
                    longDescription :   "When the Band Pass filter is enabled, this paramter adjusts the Ideal Sample Detection Algorithm to accommodate the delay posed by the filter",
                    readOnly        :   true,
                    default         :   0,
                },
                {
                    name            :   "core"+core.toString()+"IdealSampleModeAdv",
                    displayName     :   "Ideal Sample Detection Algorithm Mode",
                    longDescription :   `
    RDC_IDEAL_SAMPLE_TIME_MODE_0_AUTO_DETECT           - Computation on sin and cos
    RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_SIN    - Computation on sin only
    RDC_IDEAL_SAMPLE_TIME_MODE_1_AUTO_DETECT_ON_COS    - Computation on cos only
    RDC_IDEAL_SAMPLE_TIME_MODE_3_AUTO_DETECT_OFF       - Manual Override of Ideal Sample Time`,
                    default         :   device_peripheral.RESOLVER_idealSampleMode[0].name,
                    options         :   device_peripheral.RESOLVER_idealSampleMode,
                    readOnly        :   true,
                },
                {
                    name            :   "core"+core.toString()+"IdealSampleBottomSamplingAdv",
                    displayName     :   "Enable Bottom Sampling",
                    description     :   "enable Advanced Configurations to edit this feild",
                    longDescription :   `when Bottom Sampling is enabled, the HW Track2, ArcTan Loop run twice as fast.
    note : If SW Track2 is used, please make sure its running frequency is same as the HW Track2`,
                    readOnly        :   true,
                    default         :   false,
                },
            ],
        },

    ])

    coreConfig = coreConfig.concat([
        {
            name        :   "core"+core.toString()+"PgOffsetConfigAdv",
            displayName :   "Phase Gain Offset Configurations",
            description :   "",
            config      :   [
                {
                    name            :   "core"+core.toString()+"PgEstimationEnableAdv",
                    displayName     :   "Enable Phase Gain Estimation",
                    description     :   "enable Advanced Configurations to edit this feild. The Phase Gain Estimation Loop will be enabled",
                    longDescription :   "For the Phase Gain Estimation to be enbaled, make sure the Motor connected is in rotational state.",
                    readOnly        :   true,
                    default         :   false,
                    onChange        :   (inst, ui) => {
                        // add the other items here to be notHidden
                        if(inst["core"+core.toString()+"PgEstimationEnableAdv"] == true)
                        {
                            // ui["core"+core.toString()+"PgGlitchThresholdAdv"].hidden = false
                            ui["core"+core.toString()+"PgEstimationTrainLimitAdv"].hidden = false
                            ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = false
                            if(inst["core"+core.toString()+"PgCorrectionEnableAdv"] == true)
                            {
                                ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = false
                            }
                            else
                            {
                                ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = true
                            }
                        }
                        else
                        {
                            // ui["core"+core.toString()+"PgGlitchThresholdAdv"].hidden = true
                            ui["core"+core.toString()+"PgEstimationTrainLimitAdv"].hidden = true
                            ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = true
                        }
                    }
                },
                // {
                //     name            :   "core"+core.toString()+"PgGlitchThresholdAdv",
                //     displayName     :   "Phase Gain Estimation Glitch Threshold",
                //     description     :   "enable Advanced Configurations to edit this feild. When the Phase Gain estimation is enabled, the estimation considers samples that are over the threshold value.",
                //     hidden          :   true,
                //     readOnly        :   true,
                //     default         :   0,
                // },
                {
                    name            :   "core"+core.toString()+"PgEstimationTrainLimitAdv",
                    displayName     :   "Phase Gain Estimation Train Limit",
                    description     :   "enable Advanced Configurations to edit this feild. When the Phase Gain Estimation is enabled, the loop trains on these number of samples.",
                    hidden          :   true,
                    readOnly        :   true,
                    default         :   8,
                },
                {
                    name            :   "core"+core.toString()+"PgCorrectionEnableAdv",
                    displayName     :   "Enable Phase Gain Correction",
                    description     :   "enable Advanced Configurations to edit this feild. The Phase Gain Correction will be enabled",
                    longDescription :   "For the Phase Gain Correction to be enbaled, make sure the Motor connected is in rotational state.",
                    readOnly        :   true,
                    default         :   true,
                    onChange        :   (inst, ui) => {
                        // add the other items here to be notHidden
                        if(inst["core"+core.toString()+"PgCorrectionEnableAdv"] == true)
                        {
                            if(inst["core"+core.toString()+"PgEstimationEnableAdv"] == true)
                            {
                                ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = false
                            }
                            else
                            {
                                ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = true
                            }
                            ui["core"+core.toString()+"PgSinGainBypassValueAdv"].hidden = false
                            ui["core"+core.toString()+"PgCosGainBypassValueAdv"].hidden = false
                            ui["core"+core.toString()+"PgCosPhaseBypassValueAdv"].hidden = false
                        }
                        else
                        {
                            ui["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"].hidden = true
                            ui["core"+core.toString()+"PgSinGainBypassValueAdv"].hidden = true
                            ui["core"+core.toString()+"PgCosGainBypassValueAdv"].hidden = true
                            ui["core"+core.toString()+"PgCosPhaseBypassValueAdv"].hidden = true
                        }
                    }
                },
                {
                    name            :   "core"+core.toString()+"PgAutoPhaseGainCorrectionAdv",
                    displayName     :   "Auto Phase Gain Correction",
                    description     :   "enable Advanced Configurations to edit this feild. Auto Phase and Gain Correction is enabled, the estimated Phase Correction Value will be used for the Phase Correction.",
                    hidden          :   false,
                    readOnly        :   true,
                    default         :   false,
                    onChange        :   (inst, ui) => {
                        if (inst["core"+core.toString()+"PgAutoPhaseGainCorrectionAdv"] == true)
                        {
                            ui["core"+core.toString()+"PgSinGainBypassValueAdv"].hidden = true
                            ui["core"+core.toString()+"PgCosGainBypassValueAdv"].hidden = true
                            ui["core"+core.toString()+"PgCosPhaseBypassValueAdv"].hidden = true
                        }
                        else
                        {
                            ui["core"+core.toString()+"PgSinGainBypassValueAdv"].hidden = false
                            ui["core"+core.toString()+"PgCosGainBypassValueAdv"].hidden = false
                            ui["core"+core.toString()+"PgCosPhaseBypassValueAdv"].hidden = false
                        }
                    }
                },
                {
                    name            :   "core"+core.toString()+"PgSinGainBypassValueAdv",
                    displayName     :   "Manual Sin Gain Correction",
                    description     :   "enable Advanced Configurations to edit this feild. Manual value used for Sin Gain Correction.",
                    hidden          :   false,
                    readOnly        :   true,
                    default         :   16384,
                    onChange    : (inst, ui) => {
                        inst["manualSinGainOffset"] =  inst["core"+core.toString()+"PgSinGainBypassValueAdv"]
                    }
                },
                {
                    name            :   "core"+core.toString()+"PgCosGainBypassValueAdv",
                    displayName     :   "Manual Cos Gain Correction",
                    description     :   "enable Advanced Configurations to edit this feild. Manual value used for Cos Gain Correction.",
                    hidden          :   false,
                    readOnly        :   true,
                    default         :   16384,
                    onChange    : (inst, ui) => {
                        inst["manualCosGainOffset"] =  inst["core"+core.toString()+"PgCosGainBypassValueAdv"]
                    }
                },
                {
                    name            :   "core"+core.toString()+"PgCosPhaseBypassValueAdv",
                    displayName     :   "Manual Cos Phase Correction",
                    description     :   "enable Advanced Configurations to edit this feild. Manual value used for Cos Phase Correction.",
                    hidden          :   false,
                    readOnly        :   true,
                    default         :   0,
                    onChange    : (inst, ui) => {
                        inst["manualPhaseOffset"] =  inst["core"+core.toString()+"PgCosPhaseBypassValueAdv"]
                    }
                },
            ]
        }
    ])

    coreConfig = coreConfig.concat([
        {
            name            :   "core"+core.toString()+"track2Configurations",
            displayName     :   "Track2 Configurations",
            description     :   "",
            config          :   [
                {
                    name            :   "core"+core.toString()+"track2kiAdv",
                    displayName     :   "Ki Value",
                    readOnly        :   true,
                    default         :   64,
                    hidden          :   true,
                },
                {
                    name            :   "core"+core.toString()+"track2kffwAdv",
                    displayName     :   "Kffw Value",
                    readOnly        :   true,
                    default         :   6,
                    hidden          :   true,
                },
                {
                    name            :   "core"+core.toString()+"track2kpdivAdv",
                    displayName     :   "Kpdiv Value",
                    readOnly        :   true,
                    default         :   10,
                    hidden          :   true,
                },
                {
                    name            :   "core"+core.toString()+"track2kvelfiltAdv",
                    displayName     :   "Kvelfilt Value",
                    readOnly        :   true,
                    default         :   8,
                },
                {
                    name            :   "core"+core.toString()+"track2vboostcoefAdv",
                    displayName     :   "Vboost coefficient Value",
                    readOnly        :   true,
                    default         :   7,
                    hidden          :   true,
                },
            ]
        }
    ])

    resovlerCoreConfig = resovlerCoreConfig.concat([
        {
            name : "resovlerCore" + core.toString() + "Config",
            displayName :   "Resolver Core "+core.toString()+" Configurations",
            config  :   coreConfig,
        }
    ])
}

config = config.concat([
    {
        name  : "resovlerCoreConfigurations",
        displayName :   "Resovler Core Configurations",
        config : resovlerCoreConfig,
    }
])

let int_configs = [
    {
        name        :   "SequencerInterrupt",
        displayName :   "Sequencer Interrupt enable",
        default     :   false,
        hidden      :   false,
    },
]

for (let core = 0; core <= 1; core++)
{
    let intx_configs = [
        // RESOLVER_setupPPB(), socNumber
        {
            name        : "Core" + core.toString() + "IntEn" ,
            displayName : "Interrupt Enable for this Resolver Core",
            description : "check this box for enabling interrupt sources from this resolver core.",
            default     : false,
            hidden      : false,
            onChange    : (inst, ui)=>{
                if(inst["Core" + core.toString() + "IntEn"] == true)
                {
                    ui["ResolverCore" + core.toString() + "InterruptConfig"].hidden = false
                }
                else
                {
                    ui["ResolverCore" + core.toString() + "InterruptConfig"].hidden = true
                    inst["ResolverCore" + core.toString() + "InterruptConfig"] = []
                }
            },
        },
        {
            name: "ResolverCore" + core.toString() + "InterruptConfig",
            displayName : "Resolver Core " + core.toString() + " Interrupt Configurations",
            description : 'Select the Interrupt Sources for this Resovler Core',
            hidden      : true,
            default     : [],
            minSelections : 0,
            options     : device_peripheral.RESOLVER_InterruptSources,
        },

    ]

    int_configs = int_configs.concat([{
        name: "GROUP_INT" + core.toString(),
        displayName: "Core" + core.toString(),
        description: "Resolver Core " + core.toString() + " Interrupt Configurations",
        longDescription: "",
        config: intx_configs
    }])

}



config = config.concat([

    {
        name: "GROUP_INT",
        displayName: "INT Configurations",
        description: "Interrupt Configurations",
        longDescription: "",
        config: int_configs,
    },

])



function onValidateStatic(mod, stat)
{

}

function onValidate(inst, validation)
{

}

/*
 *  ======== filterHardware ========
 *  Control RX, TX Pin usage by the user specified dataDirection.
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    return (common.typeMatches(component.type, ["RESOLVER"]));
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    return [ "PWM0", "PWM1"  ];
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    // resources.push( pinmux.getPinRequirements(interfaceName, "ADC_R0_AIN0"));
    resources.push( pinmux.getPinRequirements("ADC_R0", "AIN0"));
    resources.push( pinmux.getPinRequirements("ADC_R0", "AIN1"));
    resources.push( pinmux.getPinRequirements("ADC_R0", "AIN2"));
    resources.push( pinmux.getPinRequirements("ADC_R0", "AIN3"));
    resources.push( pinmux.getPinRequirements("ADC_R1", "AIN0"));
    resources.push( pinmux.getPinRequirements("ADC_R1", "AIN1"));
    resources.push( pinmux.getPinRequirements("ADC_R1", "AIN2"));
    resources.push( pinmux.getPinRequirements("ADC_R1", "AIN3"));

    let resources_RES_PWM  = [];
    resources_RES_PWM.push( pinmux.getPinRequirements("RESOLVER", "PWM0"));
    resources_RES_PWM.push( pinmux.getPinRequirements("RESOLVER", "PWM1"));

    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R0", "ADC_R0_AIN0"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R0", "ADC_R0_AIN1"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R0", "ADC_R0_AIN2"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R0", "ADC_R0_AIN3"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R1", "ADC_R0_AIN0"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R1", "ADC_R0_AIN1"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R1", "ADC_R0_AIN2"));
    // resources_RES_PWM.push( pinmux.getPinRequirements("ADC_R1", "ADC_R0_AIN3"));

    // let peripheral =
    //     {
    //         name: "ADC_R",
    //         displayName: "ADC Pin Set",
    //         interfaceName: "ADC",
    //         resources: resources,
    //     };
    let peripheral2 = {
            name: interfaceName,
            displayName: "RESOLVER Excitation PWM Out",
            interfaceName: "RESOLVER",
            resources: resources_RES_PWM,
        }

    // return [peripheral2, peripheral];
    return [peripheral2];
}

let resolver_module_name = "/drivers/resolver/resolver";

let resolverModule = {
    peripheralName: "RESOLVER",
    displayName: "RESOLVER",
    defaultInstanceName: "CONFIG_RESOLVER",
    description: "Analog Digital Converter",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/resolver/templates/resolver.h.xdt",
        },
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/resolver/templates/resolver_init.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/resolver/templates/resolver_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/resolver/templates/resolver_open_close_config.c.xdt",
            driver_open: "/drivers/resolver/templates/resolver_open.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: resolver_module_name,
        },
    },

    validate    : onValidate,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
};

exports = resolverModule;