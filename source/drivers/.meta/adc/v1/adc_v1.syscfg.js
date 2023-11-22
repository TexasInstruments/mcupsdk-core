let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);
let adc_sampletime_sysclk_ns = 1000/device_peripheral.ADC_Sysclk_Mhz;

function getStaticConfigArr() {
    return system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}`).getStaticConfigArr();
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
    // ADC_setPrescaler()
    {
        name        : "adcClockPrescaler",
        displayName : "ADC Clock Prescaler",
        description : 'Clock prescaler for ADC.',
        hidden      : false,
        default     : device_peripheral.ADC_ClkPrescale[0].name,
        options     : device_peripheral.ADC_ClkPrescale,
    },
];

// ADC_setMode()
config = config.concat([
    {
        name: "adcResolutionMode",
        displayName : "ADC Resolution Mode",
        description : 'Select the ADC resolution mode.',
        hidden      : false,
        default     : device_peripheral.ADC_Resolution[0].name,
        options     : device_peripheral.ADC_Resolution,
    },
    {
        name: "adcSignalMode",
        displayName : "ADC Signal Mode",
        description : 'Select the ADC signal mode.',
        hidden      : false,
        default     : device_peripheral.ADC_SignalMode[0].name,
        options     : device_peripheral.ADC_SignalMode,
    },
]);

let soc_configs = []

for (let soci = 0; soci < 16; soci++)
{
    soc_configs = soc_configs.concat([{
        name: "GROUP_SOC" + soci.toString(),
        displayName: "SOC" + soci.toString(),
        description: "Start of Conversion " + soci.toString(),
        longDescription: "",
        config: [
            // ADC_setupSOC(), channel
            {
                name: "soc" + soci.toString() + "Channel",
                displayName : "SOC" + soci.toString() + " Channel",
                description : 'Select the channel for this SOC',
                hidden      : false,
                default     : device_peripheral.ADC_Channel[0].name,
                options     : device_peripheral.ADC_Channel,
            },
            {
                name: "GROUP_SOC" + soci.toString() + "_TRIGS",
                displayName: "SOC Triggers",
                config : [
                    // ADC_setupSOC(), trigger
                    {
                        name: "soc" + soci.toString() + "Trigger",
                        displayName : "SOC" + soci.toString() + " Trigger",
                        description : 'Select the trigger source for this SOC',
                        hidden      : false,
                        default     : device_peripheral.ADC_Trigger[0].name,
                        options     : device_peripheral.ADC_Trigger,
                    },
                    // ADC_setInterruptSOCTrigger(), trigger
                    {
                        name: "soc" + soci.toString() + "InterruptTrigger",
                        displayName : "SOC" + soci.toString() + " Interrupt Trigger",
                        description : 'Select the interrupt trigger source for this SOC. This signal is ORed with the SOC Trigger',
                        hidden      : false,
                        default     : device_peripheral.ADC_IntSOCTrigger[0].name,
                        options     : device_peripheral.ADC_IntSOCTrigger,
                    },
                ]
            },
            // ADC_setupSOC(), sampleWindow
            {
                name: "soc" + soci.toString() + "SampleWindow",
                displayName : "SOC" + soci.toString() + " Sample Window [SYSCLK Counts]",
                description : 'Select the sample window (1-512) for this SOC',
                hidden      : false,
                default     : 16,
                onChange    : (inst,ui)=>{
                    inst["soc" + soci.toString() + "SampleTime"] = adc_sampletime_sysclk_ns*(inst["soc" + soci.toString() + "SampleWindow"])
                }
            },
            {
                name: "soc" + soci.toString() + "SampleTime",
                displayName : "SOC" + soci.toString() + " Sample Time In Nanoseconds",
                description : 'Selected the sample time in ns for this SOC',
                hidden      : false,
                readOnly    : true,
                default     : adc_sampletime_sysclk_ns*16
            },

        ]
    }]);
}

config = config.concat([
    // ADC_setSOCPriority, priMode
    {
        name        : "socHighPriorityMode",
        displayName : "High Priority Mode SOCs",
        description : 'Select the range of high priority SOCs.',
        hidden      : false,
        default     : device_peripheral.ADC_PriorityMode[0].name,
        options     : device_peripheral.ADC_PriorityMode,
    },
    {
        name: "GROUP_SOC",
        displayName: "SOC Configurations",
        description: "Start of Conversion Configurations",
        longDescription: "",
        config: soc_configs
    }
])

// ADC_enableConverter() / ADC_disableConverter()
config = config.concat([
    {
        name        : "enableConverter",
        displayName : "Enable ADC Convertor",
        description : 'Enable the ADC Convertor',
        hidden      : false,
        default     : false,
    },

])

let ppb_configs = []

for (let ppbi = 1; ppbi <= 4; ppbi++)
{
    let ppbx_configs = [
        // ADC_setupPPB(), socNumber
        {
            name: "ppb" + ppbi.toString() + "SOCNumber",
            displayName : "PPB" + ppbi.toString() + " SOC Number",
            description : 'Select the SOC for this PPB',
            hidden      : false,
            default     : device_peripheral.ADC_SOCNumber[0].name,
            options     : device_peripheral.ADC_SOCNumber,
        },
        // ADC_enablePPBEvent() / ADC_disablePPBEvent, evtFlags
        {
            name: "ppb" + ppbi.toString() + "EventEnable",
            displayName : "PPB" + ppbi.toString() + " Enable Events",
            description : 'Select the EVENTs to enable for this PPB',
            hidden      : false,
            minSelections: 0,
            default     : [],
            options     : [
                {name: "ADC_EVT_TRIPHI", displayName: "ADC Event TRIP HIGH"},
                {name: "ADC_EVT_TRIPLO", displayName: "ADC Event TRIP LOW"},
                {name: "ADC_EVT_ZERO", displayName: "ADC Event ZERO CROSSING"},
            ],
        },
        // ADC_enablePPBEventInterrupt() / ADC_disablePPBEventInterrupt(), intFlags
        {
            name: "ppb" + ppbi.toString() + "EventInterruptEnable",
            displayName : "PPB" + ppbi.toString() + " Enable Interrupt Events",
            description : 'Select the Interrupt EVENTs to enable for this PPB',
            hidden      : false,
            minSelections: 0,
            default     : [],
            options     : [
                {name: "ADC_EVT_TRIPHI", displayName: "ADC Event TRIP HIGH"},
                {name: "ADC_EVT_TRIPLO", displayName: "ADC Event TRIP LOW"},
                {name: "ADC_EVT_ZERO", displayName: "ADC Event ZERO CROSSING"},
            ],
        },
        // ADC_setPPBCalibrationOffset(), offset
        {
            name: "ppb" + ppbi.toString() + "CalibrationOffset",
            displayName : "PPB" + ppbi.toString() + " Calibration Offset",
            description : 'Calibration offset applied to this PPB',
            hidden      : false,
            default     : 0
        },
        // ADC_setPPBReferenceOffset(), offset
        {
            name: "ppb" + ppbi.toString() + "ReferenceOffset",
            displayName : "PPB" + ppbi.toString() + " Reference Offset",
            description : 'Reference offset applied to this PPB',
            hidden      : false,
            default     : 0
        },
        // ADC_enablePPBTwosComplement() / ADC_disablePPBTwosComplement()
        {
            name: "ppb" + ppbi.toString() + "EnableTwosComplement",
            displayName : "PPB" + ppbi.toString() + " Enable Two's Complement",
            description : 'Select to enable the Two\'s complement mode for this PPB',
            hidden      : false,
            default     : false
        },
        // ADC_setPPBTripLimits(), tripHiLimit
        {
            name: "ppb" + ppbi.toString() + "HighTripLimit",
            displayName : "PPB" + ppbi.toString() + " High Trip Limit",
            description : 'High Trip limit for this PPB',
            hidden      : false,
            default     : 0
        },
        // ADC_setPPBTripLimits() , tripLoLimit
        {
            name: "ppb" + ppbi.toString() + "LowTripLimit",
            displayName : "PPB" + ppbi.toString() + " Low Trip Limit",
            description : 'Low Trip limit for this PPB',
            hidden      : false,
            default     : 0
        },
    ]
    // ADC_enablePPBEventCBCClear() / ADC_disablePPBEventCBCClear(), ppbNumber
    ppbx_configs = ppbx_configs.concat([
        {
            name        : "ppb" + ppbi.toString() + "EnablePPBEventCBCClear",
            displayName : "PPB" + ppbi.toString() + " Enable PPB Event CBC Clear",
            longDescription : 'Enables the automatic cycle-by-cycle clear of ADC PPB event' +
            ' flags. When enabled, the desired PPB event flags are automatically cleared' +
            ' on the next PPBxRESULT load, unless a set condition is also occurring at' +
            ' the same time, in which case the set takes precedence',
            hidden      : false,
            default     : false
        }
    ]);

    ppb_configs = ppb_configs.concat([{
        name: "GROUP_PPB" + ppbi.toString(),
        displayName: "PPB" + ppbi.toString(),
        description: "Post Processing Block " + ppbi.toString(),
        longDescription: "",
        config: ppbx_configs
    }])

}

let int_configs = [
    // ADC_setInterruptPulseMode(), pulseMode
    {
        name        : "interruptPulseMode",
        displayName : "ADC Interrupt Pulse Mode",
        description : 'Interrupt Pulse Mode for ADC EOC (end of conversion).',
        hidden      : false,
        default     : device_peripheral.ADC_PulseMode[0].name,
        options     : device_peripheral.ADC_PulseMode,
        onChange    : (inst, ui) => {
            ui["interruptCycleOffset"].hidden = false
        }
    }
]

// ADC_setInterruptCycleOffset(), cycleOffset
int_configs = int_configs.concat([
    {
        name        : "interruptCycleOffset",
        displayName : "ADC Interrupt Cycle Offset",
        description : 'Cycle offset between the negative edge of a sample ' +
                        'pulse and an early interrupt pulse being generated.',
        hidden      : false,
        default     : 0
    }
]);

for (let inti = 1; inti <= 4; inti++)
{
    int_configs = int_configs.concat([
        {
            name: "GROUP_INT" + inti.toString(),
            displayName: "INT" + inti.toString(),
            description: "ADC Interrupt " + inti.toString(),
            longDescription: "",
            config: [
                // ADC_enableInterrupt() / ADC_disableInterrupt(), adcIntNum
                {
                    name: "enableInterrupt" + inti.toString(),
                    displayName : "Enable ADC Interrupt" + inti.toString(),
                    description : 'Enable the interrupt' + inti.toString() + ' for this ADC',
                    hidden      : false,
                    default     : false
                },
                // ADC_setInterruptSource(), socNumber
                {
                    name: "interrupt" + inti.toString() + "SOCSource",
                    displayName : "Interrupt" + inti.toString() + " SOC Source",
                    description : 'Select the source for interrupt' + inti.toString() + ' of this ADC',
                    default     : device_peripheral.ADC_SOCNumber[0].name,
                    options     : device_peripheral.ADC_SOCNumber,
                },
                // ADC_enableContinuousMode()
                {
                    name: "enableInterrupt" + inti.toString() + "ContinuousMode",
                    displayName : "Enable Continue to Interrupt Mode",
                    description : 'Enable continue to interrupt mode for the interrupt' + inti.toString() + ' for this ADC',
                    hidden      : false,
                    default     : false
                },
            ]
        }
    ])
}


config = config.concat([
    {
        name: "GROUP_INT",
        displayName: "INT Configurations",
        description: "Interrupt Configurations",
        longDescription: "",
        config: int_configs
    }
])

config = config.concat([
    {
        name: "GROUP_PPB",
        displayName: "PPB Configurations",
        description: "Post Processing Blocks Configurations",
        longDescription: "",
        config: ppb_configs
    },
    {
        name: "GROUP_BURST",
        displayName: "Burst Mode",
        description: "ADC Burst Mode",
        longDescription: "",
        config: [
            // ADC_setBurstModeConfig(), trigger
            {
                name        : "burstTrigger",
                displayName : "Burst Mode Trigger Signal",
                description : 'The trigger signal for the ADC burst mode.',
                hidden      : false,
                default     : device_peripheral.ADC_Trigger[0].name,
                options     : device_peripheral.ADC_Trigger,
            },
            // ADC_setBurstModeConfig(), burstSize
            {
                name        : "burstSize",
                displayName : "Burst Size",
                description : 'The size of the burst for the ADC burst mode.',
                hidden      : false,
                default     : 1,
                options     :
                [
                    { name : 1, displayName: "Conversion bursts are 1 SOC long" },
                    { name : 2, displayName: "Conversion bursts are 2 SOC long" },
                    { name : 3, displayName: "Conversion bursts are 3 SOC long" },
                    { name : 4, displayName: "Conversion bursts are 4 SOC long" },
                    { name : 5, displayName: "Conversion bursts are 5 SOC long" },
                    { name : 6, displayName: "Conversion bursts are 6 SOC long" },
                    { name : 7, displayName: "Conversion bursts are 7 SOC long" },
                    { name : 8, displayName: "Conversion bursts are 8 SOC long" },
                    { name : 9, displayName: "Conversion bursts are 9 SOC long" },
                    { name : 10, displayName:  "Conversion bursts are 10 SOC long" },
                    { name : 11, displayName: "Conversion bursts are 11 SOC long" },
                    { name : 12, displayName: "Conversion bursts are 12 SOC long" },
                    { name : 13, displayName: "Conversion bursts are 13 SOC long" },
                    { name : 14, displayName: "Conversion bursts are 14 SOC long" },
                    { name : 15, displayName: "Conversion bursts are 15 SOC long" },
                    { name : 16, displayName: "Conversion bursts are 16 SOC long" },
                ],
            },
            // ADC_enableBurstMode() / ADC_disableBurstMode()
            {
                name        : "enableBurstMode",
                displayName : "Enable Burst Mode",
                description : 'Enable ADC burst mode.',
                hidden      : false,
                default     : false,
            },
        ]
    },
])

let globalConfig = [

    {
        name: "sampleWindowSYSCLK",
        displayName: "SYSCLK [MHz]",
        description: "This is the SYSCLK value",
        default: device_peripheral.ADC_Sysclk_Mhz,
    },
    {
        name: "synchronousModeCheck",
        displayName: "Check for Valid Synchronous Mode",
        description: "Check to see if all SOCs configured for the same trigger have the same sample window",
        default: false
    }
]

function onValidateStatic(mod, stat)
{

}

function onValidate(inst, validation) {

    //
    // Synchronous Mode
    //

    if (inst.$module.$static["synchronousModeCheck"])
    {
        //
        // Run this once
        //
        let triggers_for_adc_socs = {};

        let runStaticValidation = false;
        if (inst.$module.$instances.length > 0)
        {
            if (inst == inst.$module.$instances[0])
            {
                runStaticValidation = true;
            }
        }

        if (runStaticValidation)
        {
            let synchronousModeErrorFound = false;
            for (let instance_index in inst.$module.$instances)
            {
                let instance_obj = inst.$module.$instances[instance_index];

                console.log(instance_obj)

                if (instance_obj["adcClockPrescaler"] != inst["adcClockPrescaler"])
                {
                    validation.logError(
                        "In synchronous mode all clock prescalers must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "adcClockPrescaler");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["socHighPriorityMode"] != inst["socHighPriorityMode"])
                {
                    validation.logError(
                        "In synchronous mode all ADC high priority modes must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "socHighPriorityMode");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["enableBurstMode"] != inst["enableBurstMode"])
                {
                    validation.logError(
                        "In synchronous mode all burst modes must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "enableBurstMode");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["enableBurstMode"] && inst["enableBurstMode"])
                {
                    if (instance_obj["burstSize"] != inst["burstSize"])
                    {
                        validation.logError(
                            "In synchronous mode all burst sizes must match! " + inst.$name + " and " + instance_obj.$name +
                            " do not match!",
                            instance_obj, "burstSize");
                        synchronousModeErrorFound = true;
                    }
                    if (instance_obj["burstTrigger"] != inst["burstTrigger"])
                    {
                        validation.logError(
                            "In synchronous mode all burst triggers must match! " + inst.$name + " and " + instance_obj.$name +
                            " do not match!",
                            instance_obj, "burstTrigger");
                        synchronousModeErrorFound = true;
                    }
                }

            }

            if (!synchronousModeErrorFound)
            {
                for (let soci = 0; soci < 16; soci++)
                {
                    let socTriggers = [];
                    let socTriggerOwners = [];
                    let socSampleWindows = [];
                    for (let instance_index in inst.$module.$instances)
                    {
                        let instance_obj = inst.$module.$instances[instance_index];
                        let highPri = instance_obj.socHighPriorityMode;
                        let number_highpri_socs = device_peripheral.ADC_PriorityMode.findIndex(x => x.name == highPri);

                        if (soci < number_highpri_socs)
                        {
                            socTriggers.push(instance_obj["soc" + soci.toString() + "Trigger"])
                            socTriggerOwners.push("soc" + soci.toString() + "Trigger");
                            socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                        }
                        else
                        {
                            if (instance_obj.enableBurstMode)
                            {
                                socTriggers.push(instance_obj["burstTrigger"])
                                socTriggerOwners.push("burstTrigger");
                                socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                            }
                            else
                            {
                                socTriggers.push(instance_obj["soc" + soci.toString() + "Trigger"])
                                socTriggerOwners.push("soc" + soci.toString() + "Trigger");
                                socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                            }
                        }
                    }

                    let maxWindow = Math.max.apply(null, socSampleWindows);
                    for (let instance_index in inst.$module.$instances)
                    {
                        let instance_obj = inst.$module.$instances[instance_index];
                        for (let instance_indexCompare in inst.$module.$instances)
                        {
                            let instance_objCompare = inst.$module.$instances[instance_index];
                            if (socTriggers[instance_index] != socTriggers[instance_indexCompare])
                            {
                                validation.logError(
                                    "In synchronous mode the triggers for the same SOC numbers must match across different ADCs! " + instance_objCompare.$name + " and " + instance_obj.$name +
                                    " do not match!",
                                    instance_obj, socTriggerOwners[instance_index]);
                            }
                            if (socSampleWindows[instance_index] != socSampleWindows[instance_indexCompare])
                            {
                                validation.logError(
                                    "In synchronous mode the sampling window of the same SOC numbers must match across different ADCs! " + instance_objCompare.$name + " and " + instance_obj.$name +
                                    " do not match!" + " The maximum value found is " + maxWindow.toString(),
                                    instance_obj, "soc" + soci.toString() + "SampleWindow");
                            }
                        }
                    }
                }
            }
        }
    }

    //
    // End Synchronous Mode
    //

    //
    // Check channel selection for each SOC
    //
    for (let soci = 0; soci < 16; soci++)
    {
        let channel = inst["soc" + soci.toString() + "Channel"]
        let channel_numbers = channel.replace("ADC_CH_", "")
        let channels = channel_numbers.split("_")
        if (inst["adcSignalMode"] == "ADC_MODE_SINGLE_ENDED")
        {
            if (channels.length > 1)
            {
                validation.logError(
                    "SOC" + soci.toString() + " cannot use a differential input channel" +
                    " when the ADC module is in single ended mode",
                    inst, "soc" + soci.toString() + "Channel");
            }
        }
        else
        {
            if (channels.length < 2)
            {
                validation.logError(
                    "SOC" + soci.toString() + " cannot use a single-ended input channel" +
                    " when the ADC module is in differential mode",
                    inst, "soc" + soci.toString() + "Channel");
            }
        }
    }

    //
    // Burst Trigger warnings
    //

    let highPri = inst.socHighPriorityMode;
    let number_highpri_socs = device_peripheral.ADC_PriorityMode.findIndex(x => x.name == highPri);
    for (let soci = 0; soci < 16; soci++)
    {
        if (soci < number_highpri_socs)
        {

        }
        else
        {
            if (inst.enableBurstMode)
            {
                if (inst.burstTrigger != inst["soc" + soci.toString() + "Trigger"])
                {
                    validation.logWarning(
                        "SOC" + soci.toString() + " is configured for Round Robin. With burst mode enabled, the " +
                        " burst mode trigger will override the selected trigger for this SOC",
                        inst, "soc" + soci.toString() + "Trigger");
                }
            }
        }
    }

    //
    // Sample Window
    //
    for (let soci = 0; soci < 16; soci++)
    {
        if (inst["soc" + soci.toString() + "SampleWindow"] < 16 ||
            inst["soc" + soci.toString() + "SampleWindow"] > 512)
        {
            validation.logError(
                "SOC" + soci.toString() + " sample window must be between 16 and 512",
                inst, "soc" + soci.toString() + "SampleWindow");
        }
    }

    //
    // Cycle Offset
    //
    if (inst["interruptPulseMode"] == "ADC_PULSE_END_OF_ACQ_WIN")
    {
        if (inst["interruptCycleOffset"] < 0 || inst["interruptCycleOffset"] > 0xFFFF)
        {
            validation.logError(
                "Interupt cycle offset must be between 0 and 0xFFFF",
                inst, "interruptCycleOffset");
        }
    }

    //
    // PPB Check
    //
    for (let ppbi = 1; ppbi <= 4; ppbi++)
    {
        if (inst["ppb" + ppbi.toString() + "CalibrationOffset"] < -512 ||
            inst["ppb" + ppbi.toString() + "CalibrationOffset"] > 511)
        {
            validation.logError(
                "The calibration offset value must be between -512 and 511",
                inst,"ppb" + ppbi.toString() +  "CalibrationOffset");
        }

        if (inst["ppb" + ppbi.toString() + "ReferenceOffset"] < 0 ||
            inst["ppb" + ppbi.toString() + "CalibrationOffset"] > 0xFFFF)
        {
            validation.logError(
                "The reference offset value must be between 0 and 0xFFFF",
                inst,"ppb" + ppbi.toString() +  "CalibrationOffset");
        }

        if (inst["ppb" + ppbi.toString() + "HighTripLimit"] > 65535 ||
            inst["ppb" + ppbi.toString() + "HighTripLimit"] < -65536)
        {
            validation.logError(
                "The high trip limit value must be between 65535 and -65536",
                inst,"ppb" + ppbi.toString() +  "HighTripLimit");
        }

        if (inst["ppb" + ppbi.toString() + "LowTripLimit"] > 65535 ||
            inst["ppb" + ppbi.toString() + "LowTripLimit"] < -65536)
        {
            validation.logError(
                "The low trip limit value must be between 65535 and -65536",
                inst,"ppb" + ppbi.toString() +  "LowTripLimit");
        }
    }
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
    return (common.typeMatches(component.type, ["ADC"]));
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    return [ "AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5" ];
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN0", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN1", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN2", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN3", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN4", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN5", "ADC Input Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: "ADC Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let adcModule = {
    peripheralName: "ADC",
    displayName: "ADC",
    defaultInstanceName: "CONFIG_ADC",
    description: "Analog Digital Converter",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/adc/templates/adc.h.xdt",
        },
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/adc/templates/adc_init.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/adc/templates/adc_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/adc/templates/adc_open_close_config.c.xdt",
            driver_open: "/drivers/adc/templates/adc_open.c.xdt",
        },
    },
    moduleStatic: {
        name: "adcGlobal",
        displayName: "ADC Global",
        config: globalConfig,
        validate : onValidateStatic
    },
    validate    : onValidate,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
};

exports = adcModule;