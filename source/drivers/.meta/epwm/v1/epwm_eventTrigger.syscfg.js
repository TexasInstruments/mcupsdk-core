let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

let ADC_SOC_configs = []

function onChangeInterrupt (inst, ui)
{
    if (inst.epwmEventTrigger_enableInterrupt)
    {
        for(let uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (!configName.startsWith("GROUP_")){
                ui[configName].hidden = false;
            }
        }

        if (inst.epwmEventTrigger_interruptEventCountInitEnable == true)
        {
            ui.epwmEventTrigger_interruptEventCountInitValue.hidden = false;
        }
        else if (inst.epwmEventTrigger_interruptEventCountInitEnable == false)
        {
            ui.epwmEventTrigger_interruptEventCountInitValue.hidden = true;
            inst.epwmEventTrigger_interruptEventCountInitValue = ePWMCountInitial[0].name;
        }

        ui.epwmEventTrigger_interruptMixedSource.hidden = true;
    }
    else if (inst.epwmEventTrigger_enableInterrupt == false){
        for(let uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (!configName.startsWith("GROUP_")){
                ui[configName].hidden = true;
            }
        }

        // Set values to their defaults
        inst.epwmEventTrigger_interruptSource =  device_peripheral.EPWM_INT_TBCTR[0].name;
        inst.epwmEventTrigger_interruptMixedSource = [];
        inst.epwmEventTrigger_interruptEventCount = ePWMCount[0].name;
        inst.epwmEventTrigger_interruptEventCountInitEnable = false;
        inst.epwmEventTrigger_interruptEventCountInitValue = ePWMCountInitial[0].name;
        inst.epwmEventTrigger_interruptEventCountInitForce = false;
    }
}

function onChangeADCEnableDisable (inst, ui)
{
    //The hardcoded values here are the indices correspondign to the concerned configurables under the "ADC SOC Trigger" section

    for (let adcSocConfigIndex = 1; adcSocConfigIndex < ADC_SOC_configs.length-7; adcSocConfigIndex++)
    {
        let configName = ADC_SOC_configs[adcSocConfigIndex].name;
        if (inst.epwmEventTrigger_EPWM_SOC_A_triggerEnable)
        {
            if (adcSocConfigIndex == 4)
            {
                ui[configName].hidden = false;
                let configName2 = ADC_SOC_configs[5].name;
                let configName3 = ADC_SOC_configs[6].name;
                if (inst[configName] == true)
                {
                    ui[configName2].hidden = false;
                    ui[configName3].hidden = false;
                }
                else if (inst[configName] == false)
                {
                    ui[configName2].hidden = true;
                    ui[configName3].hidden = true;
                    // Reset values to their defaults
                    inst[configName2] = ePWMCountInitial[0].name;
                    inst[configName3] = false;
                }
            }
            else if (adcSocConfigIndex != 2 && adcSocConfigIndex != 5 && adcSocConfigIndex != 6){
                ui[configName].hidden = false;
            }
        }
        else if (inst.epwmEventTrigger_EPWM_SOC_A_triggerEnable == false)
        {
            ui[configName].hidden = true;
            // Reset values to their defaults
            inst.epwmEventTrigger_EPWM_SOC_A_triggerSource = device_peripheral.EPWM_ADCStartOfConversionSource[0].name;
            inst.epwmEventTrigger_EPWM_SOC_A_triggerEventPrescalar = ePWMCount[0].name;
            inst.epwmEventTrigger_EPWM_SOC_A_triggerEventCountInitEnable = false;
        }
    }

    for (let adcSocConfigIndex = 8; adcSocConfigIndex < ADC_SOC_configs.length; adcSocConfigIndex++)
    {
        let configName = ADC_SOC_configs[adcSocConfigIndex].name;
        if (inst.epwmEventTrigger_EPWM_SOC_B_triggerEnable)
        {
            if (adcSocConfigIndex == 11)
            {
                ui[configName].hidden = false;
                let configName2 = ADC_SOC_configs[12].name;
                let configName3 = ADC_SOC_configs[13].name;
                if (inst[configName] == true)
                {
                    ui[configName2].hidden = false;
                    ui[configName3].hidden = false;
                }
                else if (inst[configName] == false)
                {
                    ui[configName2].hidden = true;
                    ui[configName3].hidden = true;
                    // Reset values to their defaults
                    inst[configName2] = ePWMCountInitial[0].name;
                    inst[configName3] = false;
                }
            }
            else if (adcSocConfigIndex != 9 && adcSocConfigIndex != 12 && adcSocConfigIndex != 13)
            {
                ui[configName].hidden = false;
            }
        }
        else if (inst.epwmEventTrigger_EPWM_SOC_B_triggerEnable == false)
        {
            ui[configName].hidden = true;
            // Reset values to their defaults
            inst.epwmEventTrigger_EPWM_SOC_B_triggerSource = device_peripheral.EPWM_ADCStartOfConversionSource[0].name;
            inst.epwmEventTrigger_EPWM_SOC_B_triggerEventPrescalar = ePWMCount[0].name;
            inst.epwmEventTrigger_EPWM_SOC_B_triggerEventCountInitEnable = false;
        }
    }
}

function onChangeIntEventSource(inst, ui)
{
    if(inst.epwmEventTrigger_interruptSource == "EPWM_INT_TBCTR_ETINTMIX")
    {
        ui["epwmEventTrigger_interruptMixedSource"].hidden =  false;
    }
    else
    {
        ui["epwmEventTrigger_interruptMixedSource"].hidden =  true;

        // Reset values to their defaults
        inst.epwmEventTrigger_interruptMixedSource = [];
    }
}

function onChangeADCTriggerSource(inst, ui)
{
    for (let socIndex in device_peripheral.EPWM_ADCStartOfConversionType)
    {
        let soc = device_peripheral.EPWM_ADCStartOfConversionType[socIndex];
        let triggerSource= "epwmEventTrigger_" + soc.name + "_triggerSource";
        let triggerMixedSource= "epwmEventTrigger_" + soc.name + "_triggerMixedSource";

        if(inst[triggerSource] == device_peripheral.EPWM_ADCStartOfConversionSource[3].name)
        {
            ui[triggerMixedSource].hidden = false;
        }
        else
        {
            ui[triggerMixedSource].hidden = true;
            // Reset values to their defaults
            inst[triggerMixedSource] = [];
        }

    }
}

let ePWMCount = [
    { displayName: "Disabled", name: "0" },
    { displayName: "1 Event Generates Interrupt", name: "1" },
    { displayName: "2 Events Generates Interrupt", name: "2" },
    { displayName: "3 Events Generates Interrupt", name: "3" },
    { displayName: "4 Events Generates Interrupt", name: "4" },
    { displayName: "5 Events Generates Interrupt", name: "5" },
    { displayName: "6 Events Generates Interrupt", name: "6" },
    { displayName: "7 Events Generates Interrupt", name: "7" },
    { displayName: "8 Events Generates Interrupt", name: "8" },
    { displayName: "9 Events Generates Interrupt", name: "9" },
    { displayName: "10 Events Generates Interrupt", name: "10" },
    { displayName: "11 Events Generates Interrupt", name: "11" },
    { displayName: "12 Events Generates Interrupt", name: "12" },
    { displayName: "13 Events Generates Interrupt", name: "13" },
    { displayName: "14 Events Generates Interrupt", name: "14" },
    { displayName: "15 Events Generates Interrupt", name: "15" },
]

let ePWMCountInitial = [
    { displayName: "Initialize event counter to 0" , name: "0" },
    { displayName: "Initialize event counter to 1" , name: "1" },
    { displayName: "Initialize event counter to 2" , name: "2" },
    { displayName: "Initialize event counter to 3" , name: "3" },
    { displayName: "Initialize event counter to 4" , name: "4" },
    { displayName: "Initialize event counter to 5" , name: "5" },
    { displayName: "Initialize event counter to 6" , name: "6" },
    { displayName: "Initialize event counter to 7" , name: "7" },
    { displayName: "Initialize event counter to 8" , name: "8" },
    { displayName: "Initialize event counter to 9" , name: "9" },
    { displayName: "Initialize event counter to 10", name: "10" },
    { displayName: "Initialize event counter to 11", name: "11" },
    { displayName: "Initialize event counter to 12", name: "12" },
    { displayName: "Initialize event counter to 13", name: "13" },
    { displayName: "Initialize event counter to 14", name: "14" },
    { displayName: "Initialize event counter to 15", name: "15" },
]

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "epwmEventTrigger_enableInterrupt",
        displayName : "Enable EPWM Interrupt",
        description : 'Check to enable EPWM interrupt',
        hidden      : false,
        default     : false,
        onChange    : onChangeInterrupt
    },
    {
        name: "epwmEventTrigger_interruptSource",
        displayName : "Interrupt Event Sources",
        description : 'Select all events that must be enabled to generate an EPWM interrupt',
        hidden      : true,
        default     : device_peripheral.EPWM_INT_TBCTR[0].name,
        options     : device_peripheral.EPWM_INT_TBCTR,
        onChange    : onChangeIntEventSource
    },
    {
        name: "epwmEventTrigger_interruptMixedSource",
        displayName : "Interrupt Event Mixed Sources",
        description : 'Select all events that must be enabled to generate an EPWM interrupt',
        hidden      : true,
        default     : [],
        minSelections : 0,
        options     : device_peripheral.EPWM_INT_TBCTR_MixedSources,
    },
    {
        name: "epwmEventTrigger_interruptEventCount",
        displayName : "Interrupt Event Count",
        description : 'The number of events that have to occur before an interrupt is issued',
        hidden      : true,
        default     : ePWMCount[0].name,
        options     : ePWMCount
    },
    {
        name: "epwmEventTrigger_interruptEventCountInitEnable",
        displayName : "Interrupt Event Count Initial Value",
        description : 'Enables the ePWM interrupt counter to be pre-interrupt loaded with a count value after a SYNC or software force',
        hidden      : true,
        default     : false,
        onChange    : onChangeInterrupt
    },
    {
        name: "epwmEventTrigger_interruptEventCountInitValue",
        displayName : "Interrupt Event Count Initial Value",
        description : 'Value of the pre-interrupt value that is to be loaded into the interrupt event counter.',
        hidden      : true,
        default     : ePWMCountInitial[0].name,
        options     : ePWMCountInitial
    },
    {
        name: "epwmEventTrigger_interruptEventCountInitForce",
        displayName : "Force Interrupt Event Count Value",
        description : 'Forces the ePWM interrupt counter to be pre-interrupt loaded with the init count value',
        hidden      : true,
        default     : false,
    },

];

for (let socIndex in device_peripheral.EPWM_ADCStartOfConversionType)
{
    let soc = device_peripheral.EPWM_ADCStartOfConversionType[socIndex];
    let soc_name = "SOC" + soc.name.replace("EPWM_SOC_", "");
    ADC_SOC_configs = ADC_SOC_configs.concat([
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerEnable",
            displayName : soc_name + " Trigger Enable",
            description : 'Enable the SOC trigger generated by the ET submodule for ' + soc_name,
            hidden      : false,
            default     : false,
            onChange    : onChangeADCEnableDisable
        },
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerSource",
            displayName : soc_name + " Trigger Source",
            description : 'Select the SOC trigger source for ' + soc_name,
            hidden      : true,
            default     : device_peripheral.EPWM_ADCStartOfConversionSource[0].name,
            options     : device_peripheral.EPWM_ADCStartOfConversionSource,
            onChange    : onChangeADCTriggerSource
        },
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerMixedSource",
            displayName : soc_name + " Trigger Mixed Source",
            description : 'Select the SOC trigger mixed source for ' + soc_name,
            hidden      : true,
            default     : [],
            minSelections: 0,
            options     : device_peripheral.EPWM_ADCStartOfConversionMixedSource,

        },
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerEventPrescalar",
            displayName : soc_name + " Trigger Event Count",
            description : 'The number of events that have to occur before an SOC is issued',
            hidden      : true,
            default     : ePWMCount[0].name,
            options     : ePWMCount
        },

        {
            name: "epwmEventTrigger_" + soc.name + "_triggerEventCountInitEnable",
            displayName : soc_name + " Trigger Event Count Init Enable",
            description : 'Enables the ePWM SOC trigger counter to be pre-event loaded with a count value after a software force',
            hidden      : true,
            default     : false,
            onChange    : onChangeADCEnableDisable
        },
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerEventCountInitValue",
            displayName : soc_name + " Trigger Event Count Init Value",
            description : 'Value of the pre-event value that is to be loaded into the SOC trigger event counter.',
            hidden      : true,
            default     : ePWMCountInitial[0].name,
            options     : ePWMCountInitial
        },
        {
            name: "epwmEventTrigger_" + soc.name + "_triggerEventCountInitForce",
            displayName : "Force " + soc_name + " Trigger Event Count",
            description : 'Forces the ePWM SOC trigger counter to be pre-event loaded with the init count value',
            hidden      : true,
            default     : false,
        },
    ]);

}

config = config.concat([
    {
        name: "GROUP_ADCSOC",
        displayName: "ADC SOC Trigger",
        description: "",
        longDescription: "",
        config: ADC_SOC_configs
    }
]);


let epwmEventTriggerSubmodule = {
    displayName: "EPWM Event-Trigger",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_ET",
    description: "Enhanced Pulse Width Modulator Event Trigger",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = epwmEventTriggerSubmodule;