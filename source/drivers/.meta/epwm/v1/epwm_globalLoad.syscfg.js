let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

function onChangeEnableDisable(inst, ui)
{
    if (inst.epwmGlobalLoad_gld)
    {
        ui.epwmGlobalLoad_gldMode.hidden = false;
        ui.epwmGlobalLoad_gldPeriod.hidden = false;
        ui.epwmGlobalLoad_enableOneShot.hidden = false;
        ui.epwmGlobalLoad_globalePWMLoadLink.hidden = false;
        if (inst.epwmGlobalLoad_enableOneShot)
        {
            ui.epwmGlobalLoad_oneShotMode.hidden = false;
            ui.epwmGlobalLoad_oneShotForce.hidden = false;
        }
        else if (inst.epwmGlobalLoad_enableOneShot == false)
        {
            ui.epwmGlobalLoad_oneShotMode.hidden = true;
            ui.epwmGlobalLoad_oneShotForce.hidden = true;
            // Set to defalts
            inst.epwmGlobalLoad_oneShotMode = false;
            inst.epwmGlobalLoad_oneShotForce = false;
        }

    }
    else
    {
        ui.epwmGlobalLoad_gldMode.hidden = true;
        ui.epwmGlobalLoad_gldPeriod.hidden = true;
        ui.epwmGlobalLoad_enableOneShot.hidden = true;
        ui.epwmGlobalLoad_oneShotMode.hidden = true;
        ui.epwmGlobalLoad_oneShotForce.hidden = true;
        ui.epwmGlobalLoad_globalePWMLoadLink.hidden = true;

        // Set to default
        inst.epwmGlobalLoad_gldMode = device_peripheral.EPWM_GlobalLoadTrigger[0].name;
        inst.epwmGlobalLoad_gldPeriod = globalPeriodOptions[0].name;
        inst.epwmGlobalLoad_enableOneShot = false;
        inst.epwmGlobalLoad_oneShotMode = false;
        inst.epwmGlobalLoad_oneShotForce = false;
        inst.epwmGlobalLoad_globalePWMLoadLink = device_peripheral.EPWM_CurrentLink[0].name;
    }
}


let globalPeriodOptions = [
    { displayName: "Counter is disabled", name : "0" },
    { displayName: "Generate stobe on 1st event", name : "1" },
    { displayName: "Generate stobe on 2nd event", name : "2" },
    { displayName: "Generate stobe on 3rd event", name : "3" },
    { displayName: "Generate stobe on 4th event", name : "4" },
    { displayName: "Generate stobe on 5th event", name : "5" },
    { displayName: "Generate stobe on 6th event", name : "6" },
    { displayName: "Generate stobe on 7th event", name : "7" },
]

let config = [
    {
        name        : "epwmGlobalLoad_gld",
        displayName : "Enable Global Shadow to Active Load",
        description : 'All the shadow to active reload events are defined by GLDMODE bits in GLDCTL register',
        hidden      : false,
        default     : false,
        onChange    : onChangeEnableDisable
    },
    {
        name: "epwmGlobalLoad_gldMode",
        displayName : "Global Load Pulse Selection",
        description : 'The event for which there will be a Shadow to Active Mode Reload',
        hidden      : true,
        default     : device_peripheral.EPWM_GlobalLoadTrigger[0].name,
        options     : device_peripheral.EPWM_GlobalLoadTrigger,
    },
    {
        name: "epwmGlobalLoad_gldPeriod",
        displayName : "Global Load Strobe Period",
        description : 'Amount of events that need to occur before a load strobe is generated',
        hidden      : true,
        default     : globalPeriodOptions[0].name,
        options     : globalPeriodOptions,
    },
    {
        name: "epwmGlobalLoad_enableOneShot",
        displayName : "One Shot Mode",
        description : 'Enable One Shot Load Mode',
        hidden      : true,
        default     : false,
        onChange    : onChangeEnableDisable
    },
    {
        name: "epwmGlobalLoad_oneShotMode",
        displayName : "Enable Reload Event In One Shot Mode",
        description : 'Upon occurrence of a chosen load strobe, one shadow to active reload occurs',
        hidden      : true,
        default     : false,
    },
    {
        name: "epwmGlobalLoad_oneShotForce",
        displayName : "Force Load Event",
        description : 'Force one load event at the input of the event pre-scale counter',
        hidden      : true,
        default     : false,
    },
    {
        name: "epwmGlobalLoad_globalePWMLoadLink",
        displayName : "Global PWM Load Link",
        description : 'Simultaneous write to the current ePWM module GLDCTL2 register',
        hidden      : true,
        default     : device_peripheral.EPWM_CurrentLink[0].name,
        options     : device_peripheral.EPWM_CurrentLink,
    },

];



let epwmGlobalLoadSubmodule = {
    displayName: "EPWM Global Load",
    // maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_GL",
    description: "Enhanced Pulse Width Modulator Global Load Configuration",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmGlobalLoadSubmodule;