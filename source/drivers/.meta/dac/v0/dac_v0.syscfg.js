let common   = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/dac/soc/dac_${common.getSocName()}`);

let DAC_INSTANCE = [
    { name: "CSL_CONTROLSS_DAC0_U_BASE", displayName: "DAC0"},
]

function onChangeLoadMode(inst, ui)
{
    if (inst.loadMode == soc.DAC_LoadMode[0].name){
        ui.ePWMSyncSignal.hidden = true

    }
    else {
        ui.ePWMSyncSignal.hidden = false
    }
}

let numberOfDACs = 1;

/* Array of possible ePWM sync signals */
let ePWMInstances = 32
let ePWMArray = [];
for(let i = 0; i < ePWMInstances; i++) {
    ePWMArray.push()
    ePWMArray.push({ name: ""+(i+1), displayName: "EPWM"+(i)+"SYNCPER"})
}

/* Array of DAC configurables that are common across device families */
let config = [
    {
        name        : "dacBase",
        displayName : "DAC Instance",
        description : 'Instance of the DAC used.',
        hidden      : false,
        default     : DAC_INSTANCE[0].name,
        options     : DAC_INSTANCE
    },
    {
        name        : "referenceVoltage",
        displayName : "Reference Voltage",
        description : 'Sets the DAC Reference Voltage',
        hidden      : false,
        default     : soc.DAC_ReferenceVoltage[0].name,
        options     : soc.DAC_ReferenceVoltage
    },
    {
        name        : "loadMode",
        displayName : "Load Mode",
        description : 'Sets the DAC Load Mode',
        hidden      : false,
        onChange    : onChangeLoadMode,
        default     : soc.DAC_LoadMode[0].name,
        options     : soc.DAC_LoadMode
    },
    {
        name        : "ePWMSyncSignal",
        displayName : "EPWMSYNCPER Signal",
        description : 'Sets the DAC ePWMSYNC Signal',
        hidden      : true,
        default     : ePWMArray[0].name,
        options     : ePWMArray
    },
    {
        name        : "shadowValue",
        displayName : "Shadow Value",
        description : 'Set the DAC Shadow Output Value.',
        hidden      : false,
        default     : 0
    },
    {
        name        : "enableOutput",
        displayName : "Enable Output",
        description : 'Whether or not to enable the DAC output.',
        hidden      : false,
        default     : false
    },
    {
        name        : "lockedRegisters",
        displayName : "Lock DAC Registers",
        description : 'Which DAC Registers to enable.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "DAC_LOCK_CONTROL", displayName: "Control Register"},
            {name: "DAC_LOCK_SHADOW", displayName: "Shadow Register"},
            {name: "DAC_LOCK_OUTPUT", displayName: "Output Register"},
        ],

    },
];

function onValidate(inst, validation) {

    let usedDACInsts = [];
    for (let instance_index in inst.$module.$instances)
    {
        let instance_obj = inst.$module.$instances[instance_index];
        usedDACInsts.push(instance_obj.dacBase);
    }

    let duplicatesResult = common.findDuplicates(usedDACInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        let allDuplicates = "";
        for (let duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + common.stringOrEmpty(allDuplicates, ", ")
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The DAC Instance used. Duplicates: " + allDuplicates,
            inst, "dacBase");
    }

    if (inst.shadowValue < 0 || inst.shadowValue > 4095)
    {
        validation.logError(
            "Enter an integer for Shadow Value between 0 and 4095!",
            inst, "shadowValue");
    }
}

function getInterfaceName(inst)
{
    // return soc.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    // return [ "VREF0", "VREF0", "OUT" ];
}

function pinmuxRequirements(inst)
{
//    let interfaceName = getInterfaceName(inst);

//     let resources = [];

//     let peripheral = {
//         name: interfaceName,
//         displayName: "DAC Instance",
//         interfaceName: interfaceName,
//         resources: resources,
//     };

//     return [peripheral];
}

let dacModule = {
    peripheralName: "DAC",
    displayName: "DAC",
    maxInstances: numberOfDACs,
    defaultInstanceName: "CONFIG_DAC",
    description: "Digital Analog Converter",
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/dac/templates/dac.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/dac/templates/dac_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/dac/templates/dac_open_close_config.c.xdt",
            driver_open: "/drivers/dac/templates/dac_open.c.xdt",
        },
    },
    validate    : onValidate,
    // getInterfaceName,
    // getPeripheralPinNames,
    // pinmuxRequirements,
};

exports = dacModule;