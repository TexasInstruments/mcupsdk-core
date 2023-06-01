let common   = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/eqep/soc/eqep_${common.getSocName()}.syscfg.js`);

function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts){
        ui.enableInterrupt.hidden = false
        ui.interruptSources.hidden = false
    }
    else{
        ui.enableInterrupt.hidden = true
        ui.interruptSources.hidden = true
    }
}

function getStaticConfigArr() {
    return system.getScript(`/drivers/eqep/soc/eqep_${common.getSocName()}`).getStaticConfigArr();
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

/* Array of EQEP configurables that are common across device families */
let config = [];
let conditionalConfig = [];
conditionalConfig = conditionalConfig.concat([
    // setInputPolarity
    {
        name         : "inputPolarity",
        displayName  : "Input Polarity",
        description  : "Sets the polarity of the eQEP module's input signals.",
        hidden       : false,
        default      : [],
        minSelections: 0,
        options      :
        [
            {name: "invertQEPA", displayName: "Invert QEPA"},
            {name: "invertQEPB", displayName: "Invert QEPB"},
            {name: "invertIndex", displayName: "Invert Index"},
            {name: "invertStrobe", displayName: "Invert Strobe"},
        ]
    },
    // setDecoderConfig
    {
        name        : "eqepOperationMode",
        displayName : "Position Counter Source (QSRC)",
        longDescription : `Specify if quadrature signals are being provided on QEPA and QEPB,
                        if a direction signal and a clock are being provided, or if the direction
                        should be hard-wired for a single direction with QEPA used for input.`,
        hidden      : false,
        default     : device_peripheral.EQEP_CONFIG[0].name,
        options     : device_peripheral.EQEP_CONFIG.slice(0,4)
    },
    {
        name        : "eqepResolution",
        displayName : "Resolution",
        description : "Specify if both rising and falling edges should be counted or just rising edges",
        hidden      : false,
        default     : "EQEP_CONFIG_2X_RESOLUTION",
        options     :
        [
            {name: "EQEP_CONFIG_2X_RESOLUTION", displayName: "2X Resolution: Count rising and falling edge"},
            {name: "EQEP_CONFIG_1X_RESOLUTION", displayName: "1X Resolution: Count rising edge only"},
        ]
    },
    {
        name        : "eqepSwap",
        displayName : "Swap QEPA And QEPB",
        description : `Specify if the signals provided on QEPA and QEPB should be swapped
                        before being processed.`,
        hidden      : false,
        default     : device_peripheral.EQEP_CONFIG[6].name,
        options     : device_peripheral.EQEP_CONFIG.slice(6)
    },
])
conditionalConfig = conditionalConfig.concat([
    // enableDirectionChangeDuringIndex /disableDirectionChangeDuringIndex
    {
        name        : "enableDirectionChange",
        displayName : "Enable Direction Change During Index",
        description : 'Enables the index direction enhancement mode of the eQEP module.',
        hidden      : false,
        default     : false
    },
])
config = config.concat([
    {
        name            : "GROUP_DECODER",
        displayName     : "Quadrature Decoder Unit",
        description     : "",
        longDescription : "",
        config          : conditionalConfig
    },
    {
        name            : "GROUP_POSITION",
        displayName     : "Position Counter and Control Unit",
        description     : "",
        longDescription : "",
        config          : [
            // setPositionCounterConfig
            {
                name        : "positionCounterMode",
                displayName : "Position Counter Mode",
                description : "Configuration for the eQEP module position counter.",
                hidden      : false,
                default     : device_peripheral.EQEP_PositionResetMode[0].name,
                options     : device_peripheral.EQEP_PositionResetMode
            },
            {
                name        : "positionCounterMax",
                displayName : "Maximum Position",
                description : "Specifies the maximum position value.",
                hidden      : false,
                default     : 0
            },
            // setPosition
            {
                name        : "setPosition",
                displayName : "Position Counter (QPOSCNT)",
                description : "Sets the current encoder position.",
                hidden      : false,
                default     : 0
            },
            {
                name: "GROUP_LATCH",
                displayName: "Latch Mode",
                description: "",
                longDescription: "",
                collapsed: false,
                config: [
                    // setLatchMode
                    {
                        name        : "latchPositionCounter",
                        displayName : "QEP Capture Latch Mode (QCLM)",
                        description : "Specify the event that latches the position counter.",
                        hidden      : false,
                        default     : device_peripheral.EQEP_LATCH[0].name,
                        options     : device_peripheral.EQEP_LATCH.slice(0,2)
                    },
                    {
                        name        : "latchStrobePosition",
                        displayName : "Strobe Event Latch (SEL)",
                        description : "Specify which strobe event will latch the position counter into the strobe position latch register. ",
                        hidden      : false,
                        default     : device_peripheral.EQEP_LATCH[2].name,
                        options     : device_peripheral.EQEP_LATCH.slice(2,4)
                    },
                    {
                        name        : "latchIndexPosition",
                        displayName : "Index Event Latch (IEL)",
                        description : "Specify which index event will latch the position counter into the index position latch register.",
                        hidden      : false,
                        default     : device_peripheral.EQEP_LATCH[4].name,
                        options     : device_peripheral.EQEP_LATCH.slice(4)
                    },
                ]
            },
            // setPositionInitMode
            {
                name        : "positionStrobeEvents",
                displayName : "Position Strobe Events (SEI)",
                description : "Specify which strobe event will initialize the position counter.",
                hidden      : false,
                default     : device_peripheral.EQEP_INIT[0].name,
                options     : device_peripheral.EQEP_INIT.slice(0,3)
            },
            {
                name        : "positionIndexEvents",
                displayName : "Position Index Events (IEI)",
                description : "Specify which index event will initialize the position counter.",
                hidden      : false,
                default     : device_peripheral.EQEP_INIT[0].name,
                options     : device_peripheral.EQEP_INIT.slice(0,1).concat(device_peripheral.EQEP_INIT.slice(3))
            },
            // setSWPositionInit
            {
                name        : "swPositionInit",
                displayName : "Set Software Init Position Counter",
                description : 'Sets the software initialization of the encoder position counter.',
                hidden      : false,
                default     : false
            },
            // setInitialPosition
            {
                name        : "initialPosition",
                displayName : "Position Counter Init Value",
                description : 'Sets the init value for the encoder position counter.',
                hidden      : false,
                default     : 0
            },
            // enableModule / disableModule
            {
                name        : "enableModule",
                displayName : "Enable Position Counter",
                description : 'Enables the eQEP module.',
                hidden      : false,
                default     : false
            },
            {
                name: "GROUP_COMPARE",
                displayName: "Compare Configuration",
                description: "",
                longDescription: "",
                collapsed: false,
                config: [
                    // setCompareConfig [do we have a boolean to determine if we do this?]
                    {
                        name        : "compareConfig",
                        displayName : "Configure Position-Compare Unit",
                        description : 'Enables configuration of eQEP module position-compare unit.',
                        hidden      : false,
                        default     : false
                    },
                    {
                        name        : "compareSyncPulsePin",
                        displayName : "Sync Output Pin Selection (SPSEL)",
                        description : "Specify if there is a sync output pulse and which pin should be used.",
                        hidden      : false,
                        default     : device_peripheral.EQEP_COMPARE[0].name,
                        options     : device_peripheral.EQEP_COMPARE.slice(0,3)
                    },
                    {
                        name        : "compareShadowRegisterUse",
                        displayName : "Position Compare of Shadow Load",
                        description : "Specify if a shadow is enabled and when should the load should occur.",
                        hidden      : false,
                        default     : device_peripheral.EQEP_COMPARE[3].name,
                        options     : device_peripheral.EQEP_COMPARE.slice(3)
                    },
                    {
                        name        : "compareValue",
                        displayName : "Compare Value",
                        description : 'The value to which the position count value is compared for a position-compare event.',
                        hidden      : false,
                        default     : 0
                    },
                    {
                        name        : "compareCycles",
                        displayName : "Compare Cycles",
                        description : 'The width of the pulse that can be generated on a position-compare event.  It is in units of 4 SYSCLKOUT cycles.',
                        hidden      : false,
                        default     : 0
                    },
                    // enableCompare / disableCompare
                    {
                        name        : "enableCompare",
                        displayName : "Enable Position-Compare Unit",
                        description : 'Enables the eQEP module position-compare unit.',
                        hidden      : false,
                        default     : false
                    },
                ]
            },
        ]
    },
    {
        name            : "GROUP_EDGE_CAPTURE",
        displayName     : "Edge Capture Unit",
        description     : "",
        longDescription : "",
        config          : [
            // setCaptureConfig [do we have a boolean to determine if we do this?]
            {
                name        : "captureConfig",
                displayName : "Configure Edge-Capture Unit",
                description : 'Enables configuration of eQEP module edge-capture unit.',
                hidden      : false,
                default     : false
            },
            {
                name        : "capPrescale",
                displayName : "Capture Prescaler (QC Timer)",
                description : "Prescaler setting of the eQEP capture timer clk.",
                hidden      : false,
                default     : device_peripheral.EQEP_CAPCLKPrescale[0].name,
                options     : device_peripheral.EQEP_CAPCLKPrescale
            },
            {
                name        : "evntPrescale",
                displayName : "Unit Position Event Prescaler",
                description : "Prescaler setting of the unit position event.",
                hidden      : false,
                default     : device_peripheral.EQEP_UPEVNTPrescale[0].name,
                options     : device_peripheral.EQEP_UPEVNTPrescale
            },
            // enableCapture / disableCapture
            {
                name        : "enableCapture",
                displayName : "Enable Edge-Capture Unit",
                description : 'Enables the eQEP module edge-capture unit.',
                hidden      : false,
                default     : false
            },
            // enableUnitTimer / disableUnitTimer
            {
                name        : "enableUnitTimer",
                displayName : "Enable Unit Timer",
                description : 'Enables the eQEP module unit timer.',
                hidden      : false,
                default     : false
            },
            {
                name        : "unitTimerPeriod",
                displayName : "Unit Timer Period",
                description : "Period value at which a unit time-out interrupt is set.",
                hidden      : false,
                default     : 0
            },
        ]
    },

    {
        name            : "GROUP_WATCHDOG",
        displayName     : "EQEP Watchdog Timer",
        description     : "",
        longDescription : "",
        config          : [
            // enableWatchDog / disableWatchdog
            {
                name        : "enableWatchdog",
                displayName : "Enable EQEP Watchdog",
                description : "Enables the eQEP module watchdog timer.",
                hidden      : false,
                default     : false
            },
            {
                name        : "watchdogTimerPeriod",
                displayName : "EQEP Watchdog Timer Period",
                description : "Watchdog period value at which a time-out will occur if no quadrature-clock event is detected.",
                hidden      : false,
                default     : 0
            },
            // setWatchdogTimerValue
            {
                name        : "watchdogTimerValue",
                displayName : "EQEP Watchdog Timer Value",
                description : "Sets the eQEP module watchdog timer value.",
                hidden      : false,
                default     : 0
            },
        ]
    },
    {
        name            : "GROUP_INTERRUPTS",
        displayName     : "Interrupts",
        description     : "",
        longDescription : "",
        config          : [
            // use interrupt
            {
                name        : "useInterrupts",
                displayName : "Use Interrupt",
                onChange    : onChangeUseInterrupts,
                description : 'Whether or not to use Interrupt mode.',
                hidden      : false,
                default     : true
            },
            // enableInterrupt / disableInterrupt
            {
                name        : "enableInterrupt",
                displayName : "Enable Interrupt",
                description : 'Enables interrupt source.',
                hidden      : false,
                default     : false
            },
            {
                name        : "interruptSources",
                displayName : "Interrupt Sources",
                description : "The interrupt sources to be enabled.",
                hidden      : false,
                default     : [],
                minSelections: 0,
                options     : device_peripheral.EQEP_INT
            },
        ]
    },

    // setEmulationMode
    {
        name        : "emulationMode",
        displayName : "Emulation Mode",
        description : "Set the emulation mode of the eQEP module.",
        hidden      : false,
        default     : device_peripheral.EQEP_EmulationMode[0].name,
        options     : device_peripheral.EQEP_EmulationMode
    },
]);
config.push(
    // setQMAModuleMode
    {
        name        : "qmaModuleMode",
        displayName : "QMA Module Mode",
        description : "Set the quadrature mode adapter (QMA) module mode.",
        hidden      : false,
        default     : device_peripheral.EQEP_QMAMode[0].name,
        options     : device_peripheral.EQEP_QMAMode
    },
)
config.push(

    // selectSource
    {
        name        : "selectSourceA",
        displayName : "QEPA Source",
        description : "Selects the source for QEPA signal.",
        hidden      : false,
        default     : device_peripheral.EQEP_Source[0].name,
        options     : device_peripheral.EQEP_Source
    },
    {
        name        : "selectSourceB",
        displayName : "QEPB Source",
        description : "Selects the source for QEPB signal.",
        hidden      : false,
        default     : device_peripheral.EQEP_Source[0].name,
        options     : device_peripheral.EQEP_Source
    },
    {
        name        : "selectSourceIndex",
        displayName : "QEPI/Index Source",
        description : "Selects the source for Index signal.",
        hidden      : false,
        default     : device_peripheral.EQEP_Source[0].name,
        options     : device_peripheral.EQEP_Source
    },
)
config.push(
    // setStrobeSource
    {
    name        : "strobeSource",
    displayName : "Strobe Source",
    description : "Set the strobe input source of the eQEP module.",
    hidden      : false,
    default     : device_peripheral.EQEP_StrobeSource[0].name,
    options     : device_peripheral.EQEP_StrobeSource
},
)

function onValidate(inst, validation) {

    /* Validate Position Counter Max up to 32 bit unsigned int */
    if (inst.positionCounterMax < 0 || inst.positionCounterMax > 4294967295)
    {
        validation.logError(
            "Enter an integer for Position Counter Maximum between 0 and 4,294,967,295!",
            inst, "positionCounterMax");
    }

       /* Validate Position Counter Initialization Value up to 32 bit unsigned int */
       if (inst.initialPosition < 0 || inst.initialPosition > 4294967295)
       {
           validation.logError(
               "Enter an integer for Position Counter Initialization Value between 0 and 4,294,967,295!",
               inst, "initialPosition");
       }

    /* Validate Unit Timer Period up to 32 bit unsigned int */
    if (inst.unitTimerPeriod < 0 || inst.unitTimerPeriod > 4294967295)
    {
        validation.logError(
            "Enter an integer for Unit Timer Period between 0 and 4,294,967,295!",
            inst, "unitTimerPeriod");
    }

    /* Validate Encoder Position up to 32 bit unsigned int */
    if (inst.setPosition < 0 || inst.setPosition > 4294967295)
    {
        validation.logError(
            "Enter an integer for Encoder Position between 0 and 4,294,967,295!",
            inst, "setPosition");
    }

    /* Validate Watchdog Timer Period up to 16 bit unsigned int */
    if (inst.watchdogTimerPeriod < 0 || inst.watchdogTimerPeriod > 65535)
    {
        validation.logError(
            "Enter an integer for EQEP Watchdog Timer Period between 0 and 65,535!",
            inst, "watchdogTimerPeriod");
    }

    /* Validate Watchdog Timer Value up to 16 bit unsigned int */
    if (inst.watchdogTimerValue < 0 || inst.watchdogTimerValue > 65535)
    {
        validation.logError(
            "Enter an integer for EQEP Watchdog Timer Value between 0 and 65,535!",
            inst, "watchdogTimerValue");
    }

    /* Validate Compare Value up to 32 bit unsigned int */
    if (inst.compareValue < 0 || inst.compareValue > 4294967295)
    {
        validation.logError(
            "Enter an integer for Compare Value between 0 and 4,294,967,295!",
            inst, "compareValue");
    }

    /* Validate Compare Cycles up to 16 bit unsigned int */
    if (inst.compareCycles < 0 || inst.compareCycles > 65535)
    {
        validation.logError(
            "Enter an integer for Compare Cycles between 0 and 65,535!",
            inst, "compareCycles");
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
    return (common.typeMatches(component.type, ["EQEP"]));
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    return [ "A", "B", "STROBE", "INDEX" ];
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "A", "EQEPxA Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "B", "EQEPxB Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "STROBE", "EQEPx STROBE Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "INDEX", "EQEPx INDEX Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: "EQEP Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let eqep_module_name = "/drivers/eqep/eqep";

let eqepModule = {
    peripheralName: "EQEP",
    displayName: "EQEP",
    defaultInstanceName: "CONFIG_EQEP",
    description: "Enhanced Quadrature Encoder Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/eqep/templates/eqep.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/eqep/templates/eqep_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/eqep/templates/eqep_open_close_config.c.xdt",
            driver_open: "/drivers/eqep/templates/eqep_open.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: eqep_module_name,
        },
    },
    validate    : onValidate,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
};

exports = eqepModule;