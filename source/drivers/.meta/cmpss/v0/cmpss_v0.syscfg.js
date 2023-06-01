let common   = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/cmpss/soc/cmpss_${common.getSocName()}.syscfg.js`);

let numberOfInstance = 20;
let CMPSS_INSTANCE = [
    { name: "CSL_CONTROLSS_CMPSSA0_U_BASE", displayName: "CMPSSA0"},
    { name: "CSL_CONTROLSS_CMPSSA1_U_BASE", displayName: "CMPSSA1"},
    { name: "CSL_CONTROLSS_CMPSSA2_U_BASE", displayName: "CMPSSA2"},
    { name: "CSL_CONTROLSS_CMPSSA3_U_BASE", displayName: "CMPSSA3"},
    { name: "CSL_CONTROLSS_CMPSSA4_U_BASE", displayName: "CMPSSA4"},
    { name: "CSL_CONTROLSS_CMPSSA5_U_BASE", displayName: "CMPSSA5"},
    { name: "CSL_CONTROLSS_CMPSSA6_U_BASE", displayName: "CMPSSA6"},
    { name: "CSL_CONTROLSS_CMPSSA7_U_BASE", displayName: "CMPSSA7"},
    { name: "CSL_CONTROLSS_CMPSSA8_U_BASE", displayName: "CMPSSA8"},
    { name: "CSL_CONTROLSS_CMPSSA9_U_BASE", displayName: "CMPSSA9"},
    { name: "CSL_CONTROLSS_CMPSSB0_U_BASE", displayName: "CMPSSB0"},
    { name: "CSL_CONTROLSS_CMPSSB1_U_BASE", displayName: "CMPSSB1"},
    { name: "CSL_CONTROLSS_CMPSSB2_U_BASE", displayName: "CMPSSB2"},
    { name: "CSL_CONTROLSS_CMPSSB3_U_BASE", displayName: "CMPSSB3"},
    { name: "CSL_CONTROLSS_CMPSSB4_U_BASE", displayName: "CMPSSB4"},
    { name: "CSL_CONTROLSS_CMPSSB5_U_BASE", displayName: "CMPSSB5"},
    { name: "CSL_CONTROLSS_CMPSSB6_U_BASE", displayName: "CMPSSB6"},
    { name: "CSL_CONTROLSS_CMPSSB7_U_BASE", displayName: "CMPSSB7"},
    { name: "CSL_CONTROLSS_CMPSSB8_U_BASE", displayName: "CMPSSB8"},
    { name: "CSL_CONTROLSS_CMPSSB9_U_BASE", displayName: "CMPSSB9"},
]

function getDeviceADCName(adcInput)
{
    return "ADCIN" + adcInput;
}

/* Array of possible ePWM sync signals */
let ePWMInstances = 32
let ePWMArraySync = [];
let ePWMArrayBlank = [];
let ePWMArrayDE = [];
for(let i = 0; i < ePWMInstances; i++) {
    ePWMArraySync.push()
    ePWMArraySync.push({ name: ""+(i+1), displayName: "EPWM"+(i+1)+"SYNCPER" })
    ePWMArrayBlank.push()
    ePWMArrayBlank.push({ name: ""+(i+1), displayName: "EPWM"+(i+1)+"BLANK" })
    ePWMArrayDE.push()
    ePWMArrayDE.push({ name: ""+(i), displayName: "EPWM"+(i)+".DEACTIVE" })
}

/*Generate array for input signals*/
let numberOfInputSignals = 0
let asysSignalOptions = []
for(let i=0;i<numberOfInputSignals;i++){
    asysSignalOptions.push(
        {name:""+i,displayName:"Input Signal "+i},
    )
}

let highConfig =[
    // configHighComparator: Sets the configuration for the high comparator.
    {
        name        : "highCompNegative",
        displayName : "Negative Input Source",
        description : 'Specify high comparator negative input source',
        hidden      : false,
        default     : "CMPSS_INSRC_DAC",
        options     : [
            {name: "CMPSS_INSRC_DAC", displayName: "Input driven by internal DAC"},
            {name: "CMPSS_INSRC_PIN", displayName: "Input driven by external pin INL"},
        ],
    },
    {
        name        : "highCompInvert",
        displayName : "Output Is Inverted",
        description : 'High comparator output is inverted',
        hidden      : false,
        default     : false
    },
    {
        name        : "highCompAsynch",
        displayName : "Asynch OR Latch",
        description : 'Asynchronous high comparator output should be fed into an OR gate with the latched digital filter output before it is made available for CTRIPH or CTRIPOUTH.',
        hidden      : false,
        default     : false
    },
    // configOutputsHigh
    {
        name        : "highCTRIPOUT",
        displayName : "Signal Driving CTRIPOUTH",
        description : 'Specify which signal drives CTRIPOUTH',
        hidden      : false,
        default     : "CMPSS_TRIPOUT_ASYNC_COMP",
        options     : [
            {name: "CMPSS_TRIPOUT_ASYNC_COMP",  displayName: "Asynchronous comparator output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_SYNC_COMP",   displayName: "Synchronous comparator output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_FILTER",      displayName: "Filter output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_LATCH",       displayName: "Latched filter output drives CTRIPOUT"},
        ],
    },
    {
        name        : "highCTRIP",
        displayName : "Signal Driving CTRIPH",
        description : 'Specify which signal drives CTRIPH',
        hidden      : false,
        default     : "CMPSS_TRIP_ASYNC_COMP",
        options     : [
            {name: "CMPSS_TRIP_ASYNC_COMP",     displayName: "Asynchronous comparator output drives CTRIP"},
            {name: "CMPSS_TRIP_SYNC_COMP",      displayName: "Synchronous comparator output drives CTRIP"},
            {name: "CMPSS_TRIP_FILTER",         displayName: "Filter output drives CTRIP"},
            {name: "CMPSS_TRIP_LATCH",          displayName: "Latched filter output drives CTRIP"},
        ],
    },
    // setDACValueHigh
    {
        name        : "dacValHigh",
        displayName : "Set High Comparator DAC Value",
        description : 'Sets the value of the internal DAC of the high comparator.',
        hidden      : false,
        default     : 0
    },
    {
        name: "GROUP_HIGH_DIGITAL_FILTER",
        displayName: "Digital Filter Configuration",
        description: "",
        longDescription: "",
        collapsed: false,
        config: [
            // initFilterHigh
            {
                name        : "initFilterHigh",
                displayName : "Initialize Digital Filter",
                description : 'Initializes the digital filter of the high comparator.',
                hidden      : false,
                default     : false
            },
            // clearFilterLatchHigh
            {
                name        : "clearFilterLatchHigh",
                displayName : "Clear Filter Latch",
                description : 'Causes a software reset of the high comparator digital filter output latch.',
                hidden      : false,
                default     : false
            },
            // enableLatchResetOnPWSYNCHigh / disableLatchResetOnPWMSYNCHigh
            {
                name        : "latchResetHigh",
                displayName : "Reset Output Latch On EPWMSYNCPER",
                description : 'Enables reset of HIGH comparator digital filter output latch on EPWMSYNCPER',
                hidden      : false,
                default     : false
            },
            // configFilterHigh
            {
                name        : "samplePrescaleHigh",
                displayName : "Digital Filter Sample Prescale",
                description : 'The number of system clock cycles between samples.',
                hidden      : false,
                default     : 0
            },
            {
                name        : "sampleWindowHigh",
                displayName : "Digital Filter Sample Window",
                description : 'The number of FIFO samples to monitor.',
                hidden      : false,
                default     : 1
            },
            {
                name        : "thresholdHigh",
                displayName : "Digital Filter Threshold",
                description : 'The majority threshold of samples to change state.',
                hidden      : false,
                default     : 1
            },
            // configLatchOnPWMSYNC
            {
                name        : "configLatchHigh",
                displayName : "Reset Digital Filter Latch",
                description : 'Configures whether or not the digital filter latches are reset by EPWMSYNCPER in the high comparator.',
                hidden      : false,
                default     : false
            },
        ]
    },
    {
        name: "GROUP_RAMP_GENERATOR",
        displayName: "Ramp Generator Configuration",
        description: "",
        longDescription: "",
        collapsed: false,
        config: [
            // -setMaxRampValue
            {
                name        : "maxRampVal",
                displayName : "Max Ramp Generator Reference Value",
                description : 'Sets the ramp generator maximum reference value.',
                hidden      : false,
                default     : 0
            },
            // -setRampDecValue
            {
                name        : "ramDecVal",
                displayName : "Ramp Generator Decrement Value",
                description : 'Sets the ramp generator decrement value.',
                hidden      : false,
                default     : 0
            },
            // -setRampDelayValue
            {
                name        : "rampDelayVal",
                displayName : "Ramp Generator Delay Value",
                description : 'Sets the ramp generator delay value.',
                hidden      : false,
                default     : 0
            },
            {
                name        : "pwmSyncSrc",
                displayName : "EPWMSYNCPER Source Number",
                description : 'Specify the number of the EPWMSYNCPER source',
                hidden      : false,
                default     : ePWMArraySync[0].name,
                options     : ePWMArraySync
            },
            {
                name        : "useRampValShdw",
                displayName : "Ramp Generator Reset",
                description : 'Indicate whether the ramp generator should reset with the value from the ramp max reference value shadow register or with the latched ramp max reference value',
                hidden      : false,
                default     : "true",
                options     : [
                    {name: "true",  displayName: "load the ramp generator from the shadow register"},
                    {name: "false", displayName: "load the ramp generator from the latched value."},
                ],
            },
        ]
    },
]

let lowConfig =[
    // configLowComparator: Sets the configuration for the low comparator.
    {
        name        : "lowCompPositive",
        displayName : "Positive Input Source",
        description : 'Specify the low comparator positive input source',
        hidden      : false,
        default     : "CMPSS_INSRC_PIN_INL",
        options     : [
            {name: "CMPSS_INSRC_PIN_INL", displayName: "Input driven by external pin INL"},
            {name: "CMPSS_INSRC_PIN_INH", displayName: "Input driven by external pin INH"},
        ],
    },
    {
        name        : "lowCompInvert",
        displayName : "Output Is Inverted",
        description : 'Low comparator output is inverted',
        hidden      : false,
        default     : false
    },
    {
        name        : "lowCompAsynch",
        displayName : "Asynch OR Latch",
        description : 'Asynchronous low comparator output should be fed into an OR gate with the latched digital filter output before it is made available for CTRIPH or CTRIPOUTH.',
        hidden      : false,
        default     : false
    },
    // configOutputsLow
    {
        name        : "lowCTRIPOUT",
        displayName : "Signal Driving CTRIPOUTL",
        description : 'Specify which signal drives CTRIPOUTL',
        hidden      : false,
        default     : "CMPSS_TRIPOUT_ASYNC_COMP",
        options     : [
            {name: "CMPSS_TRIPOUT_ASYNC_COMP",  displayName: "Asynchronous comparator output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_SYNC_COMP",   displayName: "Synchronous comparator output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_FILTER",      displayName: "Filter output drives CTRIPOUT"},
            {name: "CMPSS_TRIPOUT_LATCH",       displayName: "Latched filter output drives CTRIPOUT"},
        ],
    },
    {
        name        : "lowCTRIP",
        displayName : "Signal Driving CTRIPL",
        description : 'Specify which signal drives CTRIPL',
        hidden      : false,
        default     : "CMPSS_TRIP_ASYNC_COMP",
        options     : [
            {name: "CMPSS_TRIP_ASYNC_COMP",     displayName: "Asynchronous comparator output drives CTRIP"},
            {name: "CMPSS_TRIP_SYNC_COMP",      displayName: "Synchronous comparator output drives CTRIP"},
            {name: "CMPSS_TRIP_FILTER",         displayName: "Filter output drives CTRIP"},
            {name: "CMPSS_TRIP_LATCH",          displayName: "Latched filter output drives CTRIP"},
        ],
    },
    // setDACValueLow
    {
        name        : "dacValLow",
        displayName : "Set Low Comparator DAC Value",
        description : 'Sets the value of the internal DAC of the low comparator.',
        hidden      : false,
        default     : 0
    },
    {
        name: "GROUP_LOW_DIGITAL_FILTER",
        displayName: "Digital Filter Configuration",
        description: "",
        longDescription: "",
        collapsed: false,
        config: [
            // initFilterLow
            {
                name        : "initFilterLow",
                displayName : "Initialize Digital Filter",
                description : 'Initializes the digital filter of the low comparator.',
                hidden      : false,
                default     : false
            },
            // clearFilterLatchLow
            {
                name        : "clearFilterLatchLow",
                displayName : "Clear Filter Latch",
                description : 'Causes a software reset of the low comparator digital filter output latch.',
                hidden      : false,
                default     : false
            },
            // enableLatchResetOnPWMSYNCLow / enableLatchResetOnPWMSYNCLow
            {
                name        : "latchResetLow",
                displayName : "Reset Output Latch On EPWMSYNCPER",
                description : 'Enables reset of LOW comparator digital filter output latch on EPWMSYNCPER',
                hidden      : false,
                default     : false
            },
            // configFilterLow
            {
                name        : "samplePrescaleLow",
                displayName : "Digital Filter Sample Prescale",
                description : 'The number of system clock cycles between samples.',
                hidden      : false,
                default     : 0
            },
            {
                name        : "sampleWindowLow",
                displayName : "Digital Filter Sample Window",
                description : 'The number of FIFO samples to monitor.',
                hidden      : false,
                default     : 1
            },
            {
                name        : "thresholdLow",
                displayName : "Digital Filter Threshold",
                description : 'The majority threshold of samples to change state.',
                hidden      : false,
                default     : 1
            },
            // configLatchOnPWMSYNC
            {
                name        : "configLatchLow",
                displayName : "Reset Digital Filter Latch",
                description : 'Configures whether or not the digital filter latches are reset by EPWMSYNCPER in the low comparator.',
                hidden      : false,
                default     : false
            },
        ]
    },
]

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name        : "cmpssBase",
        displayName : "CMPSS Instance",
        description : 'Instance of the CMPSS used.',
        hidden      : false,
        default     : CMPSS_INSTANCE[0].name,
        options     : CMPSS_INSTANCE,
        onChange    : (inst, ui) => {
            if ( (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB0_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB1_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB2_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB3_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB4_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB5_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB6_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB7_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB8_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSB9_U_BASE")
               )
            {
                ui.lowCompPositive.hidden = true
                ui.highCompNegative.hidden = true
            }
            if ( (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA0_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA1_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA2_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA3_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA4_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA5_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA6_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA7_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA8_U_BASE") ||
                 (inst.cmpssBase == "CSL_CONTROLSS_CMPSSA9_U_BASE")
               )
            {
                ui.lowCompPositive.hidden = false
                ui.highCompNegative.hidden = false
            }
        }
    },
    // enableModule / disableModule
    {
        name        : "enableModule",
        displayName : "Enable Module",
        description : 'Enables the CMPSS module.',
        hidden      : false,
        default     : false
    },
    // Group for High Comparator Configuration Functions
    {
        name: "GROUP_HIGH_COMPARATOR",
        displayName: "High Comparator Configuration",
        description: "",
        longDescription: "",
        config: highConfig
    },
    // Group for Low Comparator Configuration Functions
    {
        name: "GROUP_LOW_COMPARATOR",
        displayName: "Low Comparator Configuration",
        description: "",
        longDescription: "",
        config: lowConfig
    },
    // DAC Group
    {
        name: "GROUP_CMP_DAC",
        displayName: "DAC Configuration",
        description: "",
        longDescription: "",
	    collapsed: false,
        config: [
            // configDAC
            {
                name        : "dacValLoad",
                displayName : "DAC Value Load",
                description : 'When is DAC value loaded from shadow register',
                hidden      : false,
                default     : "CMPSS_DACVAL_SYSCLK",
                options     : [
                    {name: "CMPSS_DACVAL_SYSCLK", displayName: "DAC value updated from SYSCLK"},
                    {name: "CMPSS_DACVAL_PWMSYNC", displayName: "DAC value updated from EPWMSYNCPER"},
                ],
            },
            {
                name        : "dacRefVoltage",
                displayName : "DAC Reference Voltage",
                description : 'Specify DAC reference voltage',
                hidden      : false,
                default     : "CMPSS_DACREF_VDDA",
                options     : [
                    {name: "CMPSS_DACREF_VDDA", displayName: "VDDA is the voltage reference"},
                    {name: "CMPSS_DACREF_VDAC", displayName: "VDAC is the voltage reference"},
                ],
            },
            {
                name        : "dacValSource",
                displayName : "DAC Value Source",
                description : 'Specify DAC value source',
                hidden      : false,
                default     : "CMPSS_DACSRC_SHDW",
                options     : [
                    {name: "CMPSS_DACSRC_SHDW", displayName: "DAC value updated from shadow register"},
                    {name: "CMPSS_DACSRC_RAMP", displayName: "DAC value is updated from the ramp register"},
                ],
            },
            // enableDEmode / disableDEmode
            {
                name        : "enableDiodeEmulation",
                displayName : "Enable Diode Emulation Mode",
                description : 'Enables Diode Emulation Mode.',
                hidden      : false,
                default     : false,
                onChange    : (inst, ui) => {
                    if (inst.enableDiodeEmulation == false){
                    ui.selectDiodeEmulationSrc.hidden = true;
                    ui.dacVal2High.hidden = true;
                    ui.dacVal2Low.hidden = true;
                    }
                    if (inst.enableDiodeEmulation == true){
                    ui.selectDiodeEmulationSrc.hidden = false;
                    ui.dacVal2High.hidden = false;
                    ui.dacVal2Low.hidden = false;
                    }
                }
            },
            // selectDEACTIVESource
            {
                name        : "selectDiodeEmulationSrc",
                displayName : "Diode Emulation DEACTIVE Source",
                description : 'Sets the ePWM module DEACTIVE signal.',
                hidden      : true,
                default     : ePWMArrayDE[0].name,
                options     : ePWMArrayDE
            },
            //configHighDACShadowValue2/configLowDACShadowValue2
            {
                name        : "dacVal2High",
                displayName : "Set High Comparator DAC Value2",
                description : 'Sets the value of the internal DAC of the High comparator when DE mode is active.',
                hidden      : true,
                default     : 0
            },
            {
                name        : "dacVal2Low",
                displayName : "Set Low Comparator DAC Value2",
                description : 'Sets the value of the internal DAC of the low comparator when DE mode is active.',
                hidden      : true,
                default     : 0
            },
        ]
    },
    // setHysteresis for high comparator
    {
        name        : "hysteresisValCompH",
        displayName : "Hysteresis for High Comparator",
        description : 'Sets the the amount of hysteresis on the high comparator (COMPH).',
        hidden      : false,
        default     : "0",
        options: [
            {name:"0",displayName:"0 LSB when comparator output is 1"},
            {name:"1",displayName:"17.5 LSB when comparator output is 1"},
            {name:"2",displayName:"35 LSB when comparator output is 1"},
            {name:"3",displayName:"52.5 LSB when comparator output is 1"},
            {name:"4",displayName:"0 LSB when comparator output is 0"},
            {name:"5",displayName:"17.5 LSB when comparator output is 0"},
            {name:"6",displayName:"35 LSB when comparator output is 0"},
            {name:"7",displayName:"52.5 LSB when comparator output is 0"},
        ]
    },
    // setHysteresis for low comparator
    {
        name        : "hysteresisValCompL",
        displayName : "Hysteresis for Low Comparator",
        description : 'Sets the the amount of hysteresis on the low comparator (COMPL).',
        hidden      : false,
        default     : "0",
        options: [
            {name:"0",displayName:"0 LSB when comparator output is 1"},
            {name:"1",displayName:"17.5 LSB when comparator output is 1"},
            {name:"2",displayName:"35 LSB when comparator output is 1"},
            {name:"3",displayName:"52.5 LSB when comparator output is 1"},
            {name:"4",displayName:"0 LSB when comparator output is 0"},
            {name:"5",displayName:"17.5 LSB when comparator output is 0"},
            {name:"6",displayName:"35 LSB when comparator output is 0"},
            {name:"7",displayName:"52.5 LSB when comparator output is 0"},
        ]
    },
];

config = config.concat([
    // configBlanking
    {
        name        : "configBlanking",
        displayName : "Blanking Signal",
        description : 'Sets the ePWM module blanking signal that holds trip in reset.',
        hidden      : false,
        default     : ePWMArrayBlank[0].name,
        options     : ePWMArrayBlank
    },
    // enableBlanking / disableBlanking
    {
        name        : "enableBlanking",
        displayName : "Enable Blanking Signal",
        description : 'Enables an ePWM blanking signal to hold trip in reset.',
        hidden      : false,
        default     : false
    },
]);

function onValidate(inst, validation) {

    let usedCMPSSInsts = [];
    for (let instance_index in inst.$module.$instances)
    {
        let instance_obj = inst.$module.$instances[instance_index];
        usedCMPSSInsts.push(instance_obj.cmpssBase);
    }

    let duplicatesResult = common.findDuplicates(usedCMPSSInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        let allDuplicates = "";
        for (let duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + common.stringOrEmpty(allDuplicates, ", ")
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The CMPSS Instance used. Duplicates: " + allDuplicates,
            inst, "cmpssBase");
    }

    if (inst.dacValHigh < 0 || inst.dacValHigh > 4095)
    {
        validation.logError(
            "Enter an integer for high comparator DAC value between 0 and 4,095!",
            inst, "dacValHigh");
    }

    if (inst.dacValLow < 0 || inst.dacValLow > 4095)
    {
        validation.logError(
            "Enter an integer for low comparator DAC value between 0 and 4,095!",
            inst, "dacValLow");
    }

    if (inst.dacVal2High < 0 || inst.dacVal2High > 4095)
    {
        validation.logError(
            "Enter an integer for high comparator DAC value2 between 0 and 4,095!",
            inst, "dacVal2High");
    }

    if (inst.dacVal2Low < 0 || inst.dacVal2Low > 4095)
    {
        validation.logError(
            "Enter an integer for low comparator DAC value2 between 0 and 4,095!",
            inst, "dacVal2Low");
    }

    if (inst.maxRampVal < 0 || inst.maxRampVal > 65535)
    {
        validation.logError(
            "Enter an integer for Max Ramp Generator Reference Value between 0 and 65,535!",
            inst, "maxRampVal");
    }

    if (inst.ramDecVal < 0 || inst.ramDecVal > 8191)
    {
        validation.logError(
            "Enter an integer for Ramp Generator Decrement Value between 0 and 8,191!",
            inst, "ramDecVal");
    }

    if (inst.rampDelayVal < 0 || inst.rampDelayVal > 8191)
    {
        validation.logError(
            "Enter an integer for Ramp Generator Delay Value between 0 and 8,191!",
            inst, "rampDelayVal");
    }

    if (inst.configBlanking < 1 || inst.configBlanking > 32)
    {
        validation.logError(
            "Enter an integer for VALUE_NAME between 1 and 16!",
            inst, "configBlanking");
    }

    if (inst.samplePrescaleHigh < 0 || inst.samplePrescaleHigh > 1023)
    {
        validation.logError(
            "Enter an integer for Digital Filter Sample Prescale between 0 and 1,023!",
            inst, "samplePrescaleHigh");
    }

    if (inst.sampleWindowHigh < 1 || inst.sampleWindowHigh > 32)
    {
        validation.logError(
            "Enter an integer for Digital Filter Sample Window between 1 and 32!",
            inst, "sampleWindowHigh");
    }

    if (!(inst.thresholdHigh-1 >= ((inst.sampleWindowHigh-1)/2)) || inst.sampleWindowHigh>8191)
    {
        validation.logError(
            "Enter an integer for Digital Filter Threshold between "+Math.ceil(((inst.sampleWindowHigh-1)/2)+1)+" and 8191!",
            inst, "thresholdHigh");
    }

    if (inst.samplePrescaleLow < 0 || inst.samplePrescaleLow > 1023)
    {
        validation.logError(
            "Enter an integer for Digital Filter Sample Prescale between 0 and 1,023!",
            inst, "samplePrescaleLow");
    }

    if (inst.sampleWindowLow < 1 || inst.sampleWindowLow > 32)
    {
        validation.logError(
            "Enter an integer for Digital Filter Sample Window between 1 and 32!",
            inst, "sampleWindowLow");
    }

    if (!(inst.thresholdLow-1 >= ((inst.sampleWindowLow-1)/2)) || inst.sampleWindowLow>8191)
    {
        validation.logError(
            "Enter an integer for Digital Filter Threshold between "+Math.ceil(((inst.sampleWindowLow-1)/2)+1)+" and 8191!",
            inst, "thresholdLow");
    }

    if (inst.pwmSyncSrc < 0 || inst.pwmSyncSrc > 32)
    {
        validation.logError(
            "Enter an integer for Ramp EPWMSYNCPER source number between 1 and 16!",
            inst, "pwmSyncSrc");
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
    return (common.typeMatches(component.type, ["CMPSS"]));
}

function getInterfaceName(inst)
{
    // return device_peripheral.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    // return [ "AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5" ];
}

function pinmuxRequirements(inst)
{
    // let interfaceName = getInterfaceName(inst);

    // let resources = [];
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN0", "ADC Input Pin"));
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN1", "ADC Input Pin"));
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN2", "ADC Input Pin"));
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN3", "ADC Input Pin"));
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN4", "ADC Input Pin"));
    // resources.push( pinmux.getPinRequirements(interfaceName, "AIN5", "ADC Input Pin"));

    // let peripheral = {
    //     name: interfaceName,
    //     displayName: "ADC Instance",
    //     interfaceName: interfaceName,
    //     resources: resources,
    // };

    // return [peripheral];
}

let cmpssModule = {
    peripheralName      : "CMPSS",
    displayName         : "CMPSS",
    maxInstances        : numberOfInstance,
    defaultInstanceName : "CONFIG_CMPSS",
    description         : "Comparator Subsystem",
    filterHardware      : filterHardware,
    config              : config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/cmpss/templates/cmpss.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/cmpss/templates/cmpss_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/cmpss/templates/cmpss_open_close_config.c.xdt",
            driver_open: "/drivers/cmpss/templates/cmpss_open.c.xdt",
        },
    },
    validate            : onValidate,
    // getInterfaceName,
    // getPeripheralPinNames,
    // pinmuxRequirements,
};

exports = cmpssModule;