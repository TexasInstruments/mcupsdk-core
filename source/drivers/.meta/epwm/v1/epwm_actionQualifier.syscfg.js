let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

/* Array of configurables that are common across device families */
let UsedAQEventConfigs = [];
let UsedAQOutput_Configs = [];
let UsedAqOutput_aqEvent_Configs = [];

function onChangeUsedOutputs(inst, ui)
{
    for (let aqOutputIndex in device_peripheral.EPWM_ActionQualifierOutputModule)
    {
        let aqOutput = device_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];

        if (inst["epwmActionQualifier_" + aqOutput.name + "_shadowMode"])
        {
            ui["epwmActionQualifier_" + aqOutput.name + "_shadowEvent"].hidden = false;
        }
        else
        {
            ui["epwmActionQualifier_" + aqOutput.name + "_shadowEvent"].hidden = true;
            inst["epwmActionQualifier_" + aqOutput.name + "_shadowEvent"] = device_peripheral.EPWM_ActionQualifierLoadMode[0].name;
        }

    }

}

let config = [
    {
        name: "epwmActionQualifier_continousSwForceReloadModeGld",
        displayName : "Continuous SW Force Global Load",
        description : 'Use global load configuration for AQCSFRC',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmActionQualifier_continousSwForceReloadMode",
        displayName : "Continuous SW Force Shadow Mode",
        description : '',
        hidden      : false,
        default     : device_peripheral.EPWM_ActionQualifierContForce[0].name,
        options     : device_peripheral.EPWM_ActionQualifierContForce,
    },
    {
        name: "epwmActionQualifier_t1Source",
        displayName : "T1 Trigger Source",
        description : '',
        hidden      : false,
        default     : device_peripheral.EPWM_ActionQualifierTriggerSource[0].name,
        options     : device_peripheral.EPWM_ActionQualifierTriggerSource,
    },
    {
        name: "epwmActionQualifier_t2Source",
        displayName : "T2 Trigger Source",
        description : '',
        hidden      : false,
        default     : device_peripheral.EPWM_ActionQualifierTriggerSource[0].name,
        options     : device_peripheral.EPWM_ActionQualifierTriggerSource,
    },
];

for (let aqOutputIndex in device_peripheral.EPWM_ActionQualifierOutputModule)
{
    let aqOutput = device_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];
    let aqOutputconfigs = []

    let aqOutput_Configs = [
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_gld",
            displayName : aqOutput.displayName.replace("Output", "") + "Global Load Enable",
            description : 'Use global load configuration for ' + aqOutput.displayName.replace("output", ""),
            hidden      : false,
            default     : false,
        },
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_shadowMode",
            displayName : aqOutput.displayName.replace("Output", "") + "Shadow Mode Enable",
            description : '',
            hidden      : false,
            default     : false,
            onChange    : onChangeUsedOutputs,
        },
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_shadowEvent",
            displayName : aqOutput.displayName.replace("Output", "") + "Shadow Load Event",
            description : '',
            hidden      : true,
            default     : device_peripheral.EPWM_ActionQualifierLoadMode[0].name,
            options     : device_peripheral.EPWM_ActionQualifierLoadMode,
        },
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_onetimeSwForceAction",
            displayName : aqOutput.displayName.replace("Output", "") + "One-Time SW Force Action",
            description : '',
            hidden      : false,
            default     : device_peripheral.EPWM_ActionQualifierOutput[0].name,
            options     : device_peripheral.EPWM_ActionQualifierOutput,
        },
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_continuousSwForceAction",
            displayName : aqOutput.displayName.replace("Output", "") + "Continuous SW Force Action",
            description : '',
            hidden      : false,
            default     : device_peripheral.EPWM_ActionQualifierSWOutput[0].name,
            options     : device_peripheral.EPWM_ActionQualifierSWOutput,
        },
        {
            name: "epwmActionQualifier_" + aqOutput.name + "_usedEvents",
            displayName : "Events Configured For " + aqOutput.displayName,
            description : '',
            hidden      : false,
            default     : [],
            minSelections: 0,
            options     : device_peripheral.EPWM_ActionQualifierOutputEvent,
            onChange    : onChangeUsedOutputs,
        }

    ]

    for (let aqEventIndex in device_peripheral.EPWM_ActionQualifierOutputEvent)
    {
        let aqEvent = device_peripheral.EPWM_ActionQualifierOutputEvent[aqEventIndex];
        let aqOutput_aqEvent_config = {
            name: "epwmActionQualifier_" + aqOutput.name + "_" + aqEvent.name.replace("EPWM_AQ_OUTPUT_", ""),
            displayName : aqOutput.displayName.replace("Output", "") + aqEvent.displayName.replace("AQ OUTPUT ", ""),
            description : '',
            hidden      : false,
            default     : device_peripheral.EPWM_ActionQualifierOutput[0].name,
            options     : device_peripheral.EPWM_ActionQualifierOutput,
        };
        aqOutputconfigs.push(aqOutput_aqEvent_config);

    }

    aqOutput_Configs = aqOutput_Configs.concat([{
        name: "GROUP_AQ_" + aqOutput.name,
        displayName: aqOutput.displayName.replace("output", "") + "Event Output Configuration",
        description: "",
        longDescription: "",
        config: aqOutputconfigs
    }]);
    config = config.concat([{
            name:  "GROUP_AQ_Output_" + aqOutput.name,
            displayName: aqOutput.displayName.replace("output", "") + "Output Configuration",
            description: "",
            longDescription: "",
            config: aqOutput_Configs
    }]);

    UsedAQOutput_Configs.push(aqOutput_Configs);
}

let epwmAQSubmodule = {
    displayName: "EPWM Action Qualifier",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_AQ",
    description: "Enhanced Pulse Width Modulator Action Qualifier",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = epwmAQSubmodule;