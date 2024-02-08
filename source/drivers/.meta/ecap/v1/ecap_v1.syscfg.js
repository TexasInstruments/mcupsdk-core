let common   = system.getScript("/common");

let device_peripheral = system.getScript(`/drivers/ecap/soc/ecap_${common.getSocName()}.syscfg.js`);
let signalmonitoringunit = device_peripheral.ECAP_Monitoring_Unit;

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
        ui.qualPeriod.hidden = false
        ui.resetCounters.hidden = false

        //-------------Signal Monitoring -------------------//
        ui.trip_selection_signalMunit.hidden = false
        ui.global_strobe_selection_signalMunit.hidden = false
        for (let unit of signalmonitoringunit)
        {
            let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");
            ui["enable_monitorunit"+ unit_number].hidden = false;
        }
        //------------------------------------------------//

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
        ui.qualPeriod.hidden = true
        ui.resetCounters.hidden = true

        //-------------Signal Monitoring -------------------//
        ui.trip_selection_signalMunit.hidden = true
        ui.global_strobe_selection_signalMunit.hidden = true
        for (let unit of signalmonitoringunit)
        {
            let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");

            ui["enable_monitorunit"+ unit_number].hidden = true;
            ui["monitorSelect_"+ unit_number].hidden = true;
            ui["minValue_"+ unit_number].hidden = true;
            ui["maxValue_"+ unit_number].hidden = true;
            ui["enableSyncIn_"+ unit_number].hidden = true;
            ui["forceload_"+ unit_number].hidden = true;
            ui["loadmode_"+ unit_number].hidden = true;
            ui["shadowMinValue_"+ unit_number].hidden = true;
            ui["shadowMaxValue_"+ unit_number].hidden = true;
            ui["enableDebug_"+ unit_number].hidden = true;
        }
        //------------------------------------------------//

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
        inst.qualPeriod = device_peripheral.ECAP_QualPeriodSelect[0].name
        inst.resetCounters = false
        inst.interruptSourceCapture = []
        inst.dmaSource = device_peripheral.ECAP_DMA_TRIGGER_SOURCE[0].name

        //-------------Signal Monitoring -------------------//
        inst.trip_selection_signalMunit = device_peripheral.ECAP_MunitTripInputSelect[0].name
        inst.global_strobe_selection_signalMunit = device_peripheral.ECAP_MunitGlobalStrobeSelect[0].name
        for (let unit of signalmonitoringunit)
        {
            let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");

            inst["enable_monitorunit"+ unit_number] = false;
            inst["monitorSelect_"+ unit_number] = device_peripheral.ECAP_MonitoringTypeSelect[0].name;
            inst["minValue_"+ unit_number] = 0;
            inst["maxValue_"+ unit_number] = 0;
            inst["enableSyncIn_"+ unit_number] = false;
            inst["forceload_"+ unit_number] = false;
            inst["loadmode_"+ unit_number] = device_peripheral.ECAP_MunitloadMode[0].name;
            inst["shadowMinValue_"+ unit_number] = 0;
            inst["shadowMaxValue_"+ unit_number] = 0;
            inst["enableDebug_"+ unit_number] = false;
        }
        //------------------------------------------------//
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

let ECAP_INSTANCE = device_peripheral.ECAP_INSTANCE

let numberOfECAPs = device_peripheral.numberOfECAPs;


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
        name        : "qualPeriod",
        displayName : "Filter Out Pulses",
        description : 'Select qualification period to filter out noise.',
        hidden      : false,
        default     : device_peripheral.ECAP_QualPeriodSelect[0].name,
        options     : device_peripheral.ECAP_QualPeriodSelect
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

let signal_monitoring_unit_config = [];

function onEnableMonitorUnit(inst, ui)
{
    for (let unit of signalmonitoringunit)
    {
        let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");
        if(inst["enable_monitorunit"+ unit_number])
        {
            ui["monitorSelect_"+ unit_number].hidden = false;
            ui["minValue_"+ unit_number].hidden = false;
            ui["maxValue_"+ unit_number].hidden = false;
            ui["enableSyncIn_"+ unit_number].hidden = false;
            ui["enableDebug_"+ unit_number].hidden = false;
        }
        else
        {
            ui["monitorSelect_"+ unit_number].hidden = true;
            ui["minValue_"+ unit_number].hidden = true;
            ui["maxValue_"+ unit_number].hidden = true;
            ui["enableSyncIn_"+ unit_number].hidden = true;
            ui["forceload_"+ unit_number].hidden = true;
            ui["loadmode_"+ unit_number].hidden = true;
            ui["shadowMinValue_"+ unit_number].hidden = true;
            ui["shadowMaxValue_"+ unit_number].hidden = true;
            ui["enableDebug_"+ unit_number].hidden = true;

            // Reset values to their defaults

            inst["monitorSelect_"+ unit_number] = device_peripheral.ECAP_MonitoringTypeSelect[0].name;
            inst["minValue_"+ unit_number] = 0;
            inst["maxValue_"+ unit_number] = 0;
            inst["enableSyncIn_"+ unit_number] = false;
            inst["forceload_"+ unit_number] = false;
            inst["loadmode_"+ unit_number] = device_peripheral.ECAP_MunitloadMode[0].name;
            inst["shadowMinValue_"+ unit_number] = 0;
            inst["shadowMaxValue_"+ unit_number] = 0;
            inst["enableDebug_"+ unit_number] = false;
        }
    }
}

function onChangeSyncIn(inst, ui)
{
    for (let unit of signalmonitoringunit)
    {
        let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");
        if(inst["enableSyncIn_"+ unit_number])
        {
            ui["forceload_"+ unit_number].hidden = false;
            ui["loadmode_"+ unit_number].hidden = false;
            ui["shadowMinValue_"+ unit_number].hidden = false;
            ui["shadowMaxValue_"+ unit_number].hidden = false;
        }
        else
        {
            ui["forceload_"+ unit_number].hidden = true;
            ui["loadmode_"+ unit_number].hidden = true;
            ui["shadowMinValue_"+ unit_number].hidden = true;
            ui["shadowMaxValue_"+ unit_number].hidden = true;

            // Reset values to their defaults

            inst["forceload_"+ unit_number] = false;
            inst["loadmode_"+ unit_number] = device_peripheral.ECAP_MunitloadMode[0].name;
            inst["shadowMinValue_"+ unit_number] = 0;
            inst["shadowMaxValue_"+ unit_number] = 0;
        }
    }
}

let signal_monitoring_common_config =
[
    {
        name: "trip_selection_signalMunit",
        displayName: "Trip Selection",
        description : 'Select trip source signal to enable/ disable/ no effect signal monitoring unit',
        default: 0,
        default: device_peripheral.ECAP_MunitTripInputSelect[0].name,
        options: device_peripheral.ECAP_MunitTripInputSelect
    },
    {
        name: "global_strobe_selection_signalMunit",
        displayName: "Global Load Strobe",
        description : 'Select global load strobe to enable shadow to active loading',
        default: device_peripheral.ECAP_MunitGlobalStrobeSelect[0].name,
        options: device_peripheral.ECAP_MunitGlobalStrobeSelect
    }
]
for (let unit of signalmonitoringunit)
{
    let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");
    signal_monitoring_unit_config =  signal_monitoring_unit_config.concat ([
        {
            name: unit.name,
            displayName:   unit.name.replace(/_/g, " "),
            config:
        [
            {
                name: "enable_monitorunit"+ unit_number,
                displayName: "Enable Monitor Unit",
                description : 'Enable signal monitoring unit ' + unit_number,
                default: false,
                onChange: onEnableMonitorUnit
            },
            {
                name: "monitorSelect_"+ unit_number,
                displayName: "Select Monitoring Type Of MUNIT",
                description : 'Select Monitoring Type of MUNIT ' + unit_number,
                hidden:  true,
                default: device_peripheral.ECAP_MonitoringTypeSelect[0].name,
                options: device_peripheral.ECAP_MonitoringTypeSelect
            },
            {
                name: "minValue_"+ unit_number,
                displayName: "Minimum Value For Monitoring",
                description : 'Minimum Value For Monitoring of ' + unit_number,
                hidden:  true,
                default: 0,
            },
            {
                name: "maxValue_"+ unit_number,
                displayName: "Maximum Value For Monitoring",
                description : 'Maximum Value For Monitoring of ' + unit_number,
                hidden:  true,
                default: 0,
            },
            {
                name: "enableSyncIn_"+ unit_number,
                displayName: "Enable SyncIn",
                description : 'Enable SyncIn for ' + unit_number,
                hidden:  true,
                default: false,
                onChange: onChangeSyncIn
            },
            {
                name: "forceload_"+ unit_number,
                displayName: "Force Copy From Shadow",
                description : 'Force Copy from Shadow ' + unit_number,
                hidden:  true,
                default: false,
            },
            {
                name: "loadmode_"+ unit_number,
                displayName: "Select Shadow Load Mode",
                description : 'Select Shadow Load Mode for ' + unit_number,
                hidden:  true,
                default: device_peripheral.ECAP_MunitloadMode[0].name,
                options: device_peripheral.ECAP_MunitloadMode
            },
            {
                name: "shadowMinValue_"+ unit_number,
                displayName: "Shadow Minimum Value For Monitoring",
                description : 'Shadow Minimum Value For Monitoring of ' + unit_number,
                hidden:  true,
                default: 0,
            },
            {
                name: "shadowMaxValue_"+ unit_number,
                displayName: "Shadow Maximum Value For Monitoring",
                description : 'Shadow Maximum Value For Monitoring of ' + unit_number,
                hidden:  true,
                default: 0,
            },
            {
                name: "enableDebug_"+ unit_number,
                displayName: "Enable Debug Mode",
                description : 'Enable Debug Mode  for ' + unit_number,
                hidden:  true,
                default: false,
            },
    ]}
    ])
}

let signal_monitoring_total = signal_monitoring_common_config;
signal_monitoring_total = signal_monitoring_total.concat(signal_monitoring_unit_config);

config = config.concat([
    {
        name: "signalMonitoring",
        displayName: "Signal Monitoring Unit",
        description: "",
        config : signal_monitoring_total
    }
])


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
    for (let unit of signalmonitoringunit)
    {
        let unit_number = unit.name.replace("ECAP_MONITORING_UNIT_", "");
        /* Validate APWM Compare up to 32 bit unsigned int */
        if (inst["minValue_"+unit_number]< 0 || inst["minValue_"+unit_number] > 4294967295)
        {
            validation.logError(
                "Enter an integer for Minimum Value For Monitoring between 0 and 4,294,967,295!",
                inst, "minValue_"+unit_number);
        }
        if (inst["maxValue_"+unit_number]< 0 || inst["maxValue_"+unit_number] > 4294967295)
        {
            validation.logError(
                "Enter an integer for Maximum Value For Monitoring between 0 and 4,294,967,295!",
                inst, "maxValue_"+unit_number);
        }
        if (inst["shadowMinValue_"+unit_number]< 0 || inst["shadowMinValue_"+unit_number] > 4294967295)
        {
            validation.logError(
                "Enter an integer for Shadow Minimum Value For Monitoring between 0 and 4,294,967,295!",
                inst, "shadowMinValue_"+unit_number);
        }
        if (inst["shadowMaxValue_"+unit_number]< 0 || inst["shadowMaxValue_"+unit_number] > 4294967295)
        {
            validation.logError(
                "Enter an integer for Shadow Maximum Value For Monitoring between 0 and 4,294,967,295!",
                inst, "shadowMaxValue_"+unit_number);
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