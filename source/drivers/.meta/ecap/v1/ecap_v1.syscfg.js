let common   = system.getScript("/common");

let device_peripheral = system.getScript("/drivers/ecap/soc/ecap_am263x.syscfg.js");

function onChangeECAPMode(inst, ui)
{
    if (inst.ecapMode == "CAPTURE"){
        //CAPTURE Options
        ui.eventPrescaler.hidden = false
        ui.eventOnePolarity.hidden = false
        ui.eventTwoPolarity.hidden = false
        ui.eventThreePolarity.hidden = false
        ui.eventFourPolarity.hidden = false
        ui.captureMode.hidden = false
        ui.dmaSource.hidden = false;
        ui.dmaSourceAPWM.hidden = true;
        if (inst.useInterrupts){
            ui.interruptSourceCapture.hidden = false
            ui.interruptSourceAPWM.hidden = true
        }
        ui.eventStop.hidden = false
        ui.counterResetOnEvent.hidden = false
        ui.reArm.hidden = false
        ui.ecapInput.hidden = false
        ui.resetCounters.hidden = false

        //APWM options
        ui.apwmPolarity.hidden = true
        ui.apwmPeriod.hidden = true
        ui.apwmCompare.hidden = true

        //Reset to default values
        inst.ecap_AdcSoCtriggerSourceAPWM = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[4].name
        inst.apwmPolarity = device_peripheral.ECAP_APWMPolarity[0].name
        inst.apwmPeriod = 0
        inst.apwmCompare = 0
        inst.interruptSourceAPWM = []
        inst.dmaSourceAPWM = device_peripheral.ECAP_DMA_TRIGGER_SOURCE[4].name
    }
    else if(inst.ecapMode == "APWM"){
        //CAPTURE options
        ui.eventPrescaler.hidden = true
        ui.eventOnePolarity.hidden = true
        ui.eventTwoPolarity.hidden = true
        ui.eventThreePolarity.hidden = true
        ui.eventFourPolarity.hidden = true
        ui.captureMode.hidden = true
        ui.dmaSource.hidden = true;
        ui.dmaSourceAPWM.hidden = false;
        if (inst.useInterrupts){
            ui.interruptSourceCapture.hidden = true
            ui.interruptSourceAPWM.hidden = false
        }
        ui.eventStop.hidden = true
        ui.counterResetOnEvent.hidden = true
        ui.reArm.hidden = true
        ui.ecapInput.hidden = true
        ui.resetCounters.hidden = true

        //APWM options
        ui.apwmPeriod.hidden = false
        ui.apwmPolarity.hidden = false
        ui.apwmCompare.hidden = false

        //Reset to default values
        inst.eventPrescaler = 0
        inst.eventOnePolarity = device_peripheral.ECAP_EventPolarity[0].name
        inst.eventTwoPolarity = device_peripheral.ECAP_EventPolarity[0].name
        inst.eventThreePolarity = device_peripheral.ECAP_EventPolarity[0].name
        inst.eventFourPolarity = device_peripheral.ECAP_EventPolarity[0].name
        inst.ecap_AdcSoCtriggerSourceCapture = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[0].name
        inst.eventStop = device_peripheral.ECAP_Events[0].name
        inst.counterResetOnEvent = []
        inst.reArm = false
        inst.ecapInput = defaultInput
        inst.resetCounters = false
        inst.interruptSourceCapture = []
        inst.dmaSource = device_peripheral.ECAP_DMA_TRIGGER_SOURCE[0].name
    }
    inst.ecap_AdcSoCtriggerEnable = false
    ui.ecap_AdcSoCtriggerSourceCapture.hidden = true
    ui.ecap_AdcSoCtriggerSourceAPWM.hidden = true
}

function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts){
        ui.enableInterrupt.hidden = false
        if(inst.ecapMode == "CAPTURE"){
            ui.interruptSourceCapture.hidden = false
            ui.interruptSourceAPWM.hidden = true

            // Reset values to their defaults
            inst.interruptSourceAPWM = []
        }
        else if(inst.ecapMode == "APWM"){
            ui.interruptSourceCapture.hidden = true
            ui.interruptSourceAPWM.hidden = false

            // Reset values to their defaults
            inst.interruptSourceCapture = []
        }

    }
    else{
        ui.enableInterrupt.hidden = true
        ui.interruptSourceCapture.hidden = true
        ui.interruptSourceAPWM.hidden = true

        // Reset values to their defaults
        inst.interruptSourceCapture = []
        inst.interruptSourceAPWM = []
    }
}

function onChangeADCEnableDisable(inst, ui)
{
    if (inst.ecap_AdcSoCtriggerEnable){

        if(inst.ecapMode == "CAPTURE"){
            ui.ecap_AdcSoCtriggerSourceCapture.hidden = false
            ui.ecap_AdcSoCtriggerSourceAPWM.hidden = true
            // Reset values to their defaults
            inst.ecap_AdcSoCtriggerSourceAPWM = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[4].name
        }
        else if(inst.ecapMode == "APWM"){
            ui.ecap_AdcSoCtriggerSourceCapture.hidden = true
            ui.ecap_AdcSoCtriggerSourceAPWM.hidden = false
            // Reset values to their defaults
            inst.ecap_AdcSoCtriggerSourceCapture = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[0].name
        }

    }
    else{
        ui.ecap_AdcSoCtriggerSourceCapture.hidden = true
        ui.ecap_AdcSoCtriggerSourceAPWM.hidden = true

        //Reset to default values
        inst.ecap_AdcSoCtriggerSourceCapture = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[0].name
        inst.ecap_AdcSoCtriggerSourceAPWM = device_peripheral.ECAP_ADC_TRIGGER_SOURCE[4].name
    }
}

let ECAP_INSTANCE = [
    { name: "CSL_CONTROLSS_ECAP0_U_BASE", displayName: "ECAP0"},
    { name: "CSL_CONTROLSS_ECAP1_U_BASE", displayName: "ECAP1"},
    { name: "CSL_CONTROLSS_ECAP2_U_BASE", displayName: "ECAP2"},
    { name: "CSL_CONTROLSS_ECAP3_U_BASE", displayName: "ECAP3"},
    { name: "CSL_CONTROLSS_ECAP4_U_BASE", displayName: "ECAP4"},
    { name: "CSL_CONTROLSS_ECAP5_U_BASE", displayName: "ECAP5"},
    { name: "CSL_CONTROLSS_ECAP6_U_BASE", displayName: "ECAP6"},
    { name: "CSL_CONTROLSS_ECAP7_U_BASE", displayName: "ECAP7"},
    { name: "CSL_CONTROLSS_ECAP8_U_BASE", displayName: "ECAP8"},
    { name: "CSL_CONTROLSS_ECAP9_U_BASE", displayName: "ECAP9"},
]

let numberOfECAPs = 10;


/* Array of CAN configurables that are common across device families */
let config = [
    {
        name        : "ecapBase",
        displayName : "ECAP Instance",
        description : 'Instance of the ECAP used.',
        hidden      : false,
        default     : ECAP_INSTANCE[0].name,
        options     : ECAP_INSTANCE
    },
    {
        name        : "emulationMode",
        displayName : "Emulation Mode",
        description : 'Configures emulation mode.',
        hidden      : false,
        default     : device_peripheral.ECAP_EmulationMode[0].name,
        options     : device_peripheral.ECAP_EmulationMode
    },
    {
        name        : "ecapMode",
        displayName : "ECAP Mode",
        onChange    : onChangeECAPMode,
        default     : "CAPTURE",
        options     :
        [
            {name: "CAPTURE", displayName: "Capture"},
            {name: "APWM", displayName: "APWM"},
        ]
    },
    {
        name        : "captureMode",
        displayName : "Capture Mode",
        description : 'Sets the capture mode.',
        hidden      : false,
        default     : device_peripheral.ECAP_CaptureMode[0].name,
        options     : device_peripheral.ECAP_CaptureMode
    },
    {
        name        : "eventStop",
        displayName : "Capture Stops At Event",
        description : 'Select the event number at which the counter stops or wraps',
        hidden      : false,
        default     : device_peripheral.ECAP_Events[0].name,
        options     : device_peripheral.ECAP_Events
    },
    {
        name        : "eventPrescaler",
        displayName : "Event Prescaler",
        description : 'Sets the input prescaler.',
        hidden      : false,
        default     : 0
    },
    {
        name        : "eventOnePolarity",
        displayName : "Event 1 Polarity",
        description : 'Sets the Capture event polarity for Event 1.',
        hidden      : false,
        default     : device_peripheral.ECAP_EventPolarity[0].name,
        options     : device_peripheral.ECAP_EventPolarity
    },
    {
        name        : "eventTwoPolarity",
        displayName : "Event 2 Polarity",
        description : 'Sets the Capture event polarity for Event 2.',
        hidden      : false,
        default     : device_peripheral.ECAP_EventPolarity[0].name,
        options     : device_peripheral.ECAP_EventPolarity
    },
    {
        name        : "eventThreePolarity",
        displayName : "Event 3 Polarity",
        description : 'Sets the Capture event polarity for Event 3.',
        hidden      : false,
        default     : device_peripheral.ECAP_EventPolarity[0].name,
        options     : device_peripheral.ECAP_EventPolarity
    },
    {
        name        : "eventFourPolarity",
        displayName : "Event 4 Polarity",
        description : 'Sets the Capture event polarity for Event 4.',
        hidden      : false,
        default     : device_peripheral.ECAP_EventPolarity[0].name,
        options     : device_peripheral.ECAP_EventPolarity
    },
    {
        name        : "useInterrupts",
        displayName : "Use Interrupt",
        onChange    : onChangeUseInterrupts,
        description : 'Whether or not to use Interrupt mode.',
        hidden      : false,
        default     : true
    },
    {
        name        : "enableInterrupt",
        displayName : "Enable Interrupt",
        description : 'Enables interrupt source.',
        hidden      : false,
        default     : false
    },
    {
        name        : "interruptSourceCapture",
        displayName : "Interrupt Source",
        description : 'Select interrupt source.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "ECAP_ISR_SOURCE_CAPTURE_EVENT_1", displayName: "Event 1 ISR source"},
            {name: "ECAP_ISR_SOURCE_CAPTURE_EVENT_2", displayName: "Event 2 ISR source"},
            {name: "ECAP_ISR_SOURCE_CAPTURE_EVENT_3", displayName: "Event 3 ISR source"},
            {name: "ECAP_ISR_SOURCE_CAPTURE_EVENT_4", displayName: "Event 4 ISR source"},
            {name: "ECAP_ISR_SOURCE_COUNTER_OVERFLOW", displayName: "Counter overflow ISR source"},
        ],
    },
    {
        name        : "interruptSourceAPWM",
        displayName : "Interrupt Source",
        description : 'Select interrupt source.',
        hidden      : true,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "ECAP_ISR_SOURCE_COUNTER_PERIOD", displayName: "Counter equals period ISR source"},
            {name: "ECAP_ISR_SOURCE_COUNTER_COMPARE", displayName: "Counter equals compare ISR source"},
        ],
    },
    {
        name        : "counterResetOnEvent",
        displayName : "Enable Counter Reset",
        description : 'Enables counter reset on an event.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : device_peripheral.ECAP_Events

    },
    {
        name        : "phaseShiftCount",
        displayName : "Phase Shift Count",
        description : 'Sets a phase shift value count.',
        hidden      : false,
        default     : 0
    },
    {
        name        : "enableLoadCounter",
        displayName : "Enable Load Counter",
        description : 'Enable counter loading with phase shift value.',
        hidden      : false,
        default     : false
    },
    {
        name        : "loadCounter",
        displayName : "Load Counter",
        description : 'Load time stamp counter.',
        hidden      : false,
        default     : false
    },
    {
        name        : "syncOutMode",
        displayName : "Sync Out Mode",
        description : 'Configures Sync out signal mode.',
        hidden      : false,
        default     : device_peripheral.ECAP_SyncOutMode[0].name,
        options     : device_peripheral.ECAP_SyncOutMode
    },
    {
        name        : "apwmPolarity",
        displayName : "APWM Polarity",
        description : 'Set eCAP APWM polarity.',
        hidden      : true,
        default     : device_peripheral.ECAP_APWMPolarity[0].name,
        options     : device_peripheral.ECAP_APWMPolarity
    },
    {
        name        : "apwmPeriod",
        displayName : "APWM Period",
        description : 'Set eCAP APWM period.',
        hidden      : true,
        default     : 0
    },
    {
        name        : "apwmCompare",
        displayName : "APWM Compare",
        description : 'Set eCAP APWM on or off time count.',
        hidden      : true,
        default     : 0
    },
];

let defaultInput = "ECAP_INPUT_INPUTXBAR1"

config.push(
    {
        name        : "ecapInput",
        displayName : "ECAP Input",
        description : 'Select eCAP input.',
        hidden      : false,
        default     : defaultInput,
        options     : device_peripheral.ECAP_InputCaptureSignals
    },
    {
        name        : "resetCounters",
        displayName : "Reset Counters",
        description : 'Resets eCAP counters and flags.',
        hidden      : false,
        default     : false
    },
    {
        name        : "useDMA",
        displayName : "Use DMA Source",
        description : 'Enables use of DMA Source.',
        hidden      : false,
        default     : false
    },
    {
        name        : "dmaSource",
        displayName : "DMA Source",
        description : 'Sets the eCAP DMA source in Capture Mode.',
        hidden      : false,
        default     : device_peripheral.ECAP_DMA_TRIGGER_SOURCE[0].name,
        options     : [device_peripheral.ECAP_DMA_TRIGGER_SOURCE[0],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[1],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[2],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[3],
                      ]
    },
    {
        name        : "dmaSourceAPWM",
        displayName : "DMA Source",
        description : 'Sets the eCAP DMA source in APWM mode.',
        hidden      : true,
        default     : device_peripheral.ECAP_DMA_TRIGGER_SOURCE[4].name,
        options     : [device_peripheral.ECAP_DMA_TRIGGER_SOURCE[4],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[5],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[6],
                       device_peripheral.ECAP_DMA_TRIGGER_SOURCE[7],
                      ]
    },
    {
        name: "ecap_AdcSoCtriggerEnable",
        displayName : "ADC SOC Trigger Enable",
        description : 'Enable the ADC SOC trigger',
        hidden      : false,
        default     : false,
        onChange    : onChangeADCEnableDisable
    },
    {
        name: "ecap_AdcSoCtriggerSourceCapture",
        displayName : "ADC SOC Trigger Source",
        description : 'Select the ADC SOC trigger source in Capture mode',
        hidden      : true,
        default     : device_peripheral.ECAP_ADC_TRIGGER_SOURCE[0].name,
        options     : [device_peripheral.ECAP_ADC_TRIGGER_SOURCE[0],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[1],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[2],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[3],
                    ]
    },
    {
        name: "ecap_AdcSoCtriggerSourceAPWM",
        displayName : "ADC SOC Trigger Source",
        description : 'Select the ADC SOC trigger source in APWM mode',
        hidden      : true,
        default     : device_peripheral.ECAP_ADC_TRIGGER_SOURCE[4].name,
        options     : [device_peripheral.ECAP_ADC_TRIGGER_SOURCE[4],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[5],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[6],
                       device_peripheral.ECAP_ADC_TRIGGER_SOURCE[7],
                       ]
    },
)

config.push(
    {
        name        : "syncInPulseSource",
        displayName : "Sync-In Pulse Source",
        description : 'Set up the source for sync-in pulse.',
        hidden      : false,
        default     : device_peripheral.ECAP_SyncInPulseSource[0].name,
        options     : device_peripheral.ECAP_SyncInPulseSource
    },
)

config.push(
    {
        name        : "reArm",
        displayName : "Re-arm ECAP",
        description : 'Enable counter loading with phase shift value.',
        hidden      : false,
        default     : false
    },
)

function onValidate(inst, validation) {

    let usedECAPInsts = [];
    for (let instance_index in inst.$module.$instances)
    {
        let instance_obj = inst.$module.$instances[instance_index];
        usedECAPInsts.push(instance_obj.ecapBase);
    }

    let duplicatesResult = common.findDuplicates(usedECAPInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        let allDuplicates = "";
        for (let duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + common.stringOrEmpty(allDuplicates, ", ")
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The ECAP Instance used. Duplicates: " + allDuplicates,
            inst, "ecapBase");
    }

    /* Validate Event Prescaler up to 16 bit unsigned int */
    if (inst.eventPrescaler < 0 || inst.eventPrescaler > 65535)
    {
        validation.logError(
            "Enter an integer for Event Prescaler between 0 and 65,535!",
            inst, "eventPrescaler");
    }

    /* Validate Phase Shift Count up to 32 bit unsigned int */
    if (inst.phaseShiftCount < 0 || inst.phaseShiftCount > 4294967295)
    {
        validation.logError(
            "Enter an integer for Phase Shift Count between 0 and 4,294,967,295!",
            inst, "phaseShiftCount");
    }

    /* Validate APWM Period up to 32 bit unsigned int */
    if (inst.apwmPeriod < 0 || inst.apwmPeriod > 4294967295)
    {
        validation.logError(
            "Enter an integer for APWM Period between 0 and 4,294,967,295!",
            inst, "apwmPeriod");
    }

    /* Validate APWM Compare up to 32 bit unsigned int */
    if (inst.apwmCompare < 0 || inst.apwmCompare > 4294967295)
    {
        validation.logError(
            "Enter an integer for APWM Compare between 0 and 4,294,967,295!",
            inst, "apwmCompare");
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
    return (common.typeMatches(component.type, ["ECAP"]));
}
//module static on a variable, null if not needed, otherwise the object we have now.
let ecapModule = {
    peripheralName: "ECAP",
    displayName: "ECAP",
    maxInstances: numberOfECAPs,
    defaultInstanceName: "CONFIG_ECAP",
    description: "Enhanced Capture",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/ecap/templates/ecap.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/ecap/templates/ecap_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/ecap/templates/ecap_open_close_config.c.xdt",
            driver_open: "/drivers/ecap/templates/ecap_open.c.xdt",
        },
    },
    validate    : onValidate,
};

exports = ecapModule;