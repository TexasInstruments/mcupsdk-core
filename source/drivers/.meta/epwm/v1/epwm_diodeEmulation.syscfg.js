let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);


function onClickEnable(inst, ui)
{
    if (inst.epwmDE_enableMode)
    {
        ui["epwmDE_selectMode"].hidden = false;
        ui["epwmDE_reEntryDelay"].hidden = false;
        ui["epwmDE_Frc"].hidden = false;
        ui["epwmDE_TripMonitorMode"].hidden = false;

        for(let uiConfigIndex = 3; uiConfigIndex < 5; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_DECOMPSEL")
                ||  configName.startsWith("GROUP_DEACTCTL"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                for(let i = 0; i < groupLen; i++)  //Displaying only the ACTIVE set
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = false;
                    }
            }
        }
    }

    else
    {
        ui["epwmDE_selectMode"].hidden = true;
        ui["epwmDE_reEntryDelay"].hidden = true;
        ui["epwmDE_Frc"].hidden = true;
        ui["epwmDE_TripMonitorMode"].hidden = true;

        for(let uiConfigIndex = 3; uiConfigIndex < 5; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_DECOMPSEL")
                ||  configName.startsWith("GROUP_DEACTCTL"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                for(let i = 0; i < groupLen; i++)
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = true;
                    }
            }
        }
    }
}

function onClickMonMode(inst, ui)
{
    if (inst.epwmDE_TripMonitorMode)
    {
        ui["epwmDE_Threshold"].hidden = false;
        for(let uiConfigIndex = 8; uiConfigIndex < 9; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_DEMONSTEP"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                for(let i = 0; i < groupLen; i++)
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = false;
                    }
            }
        }
    }
    else
    {
        ui["epwmDE_Threshold"].hidden = true;
        for(let uiConfigIndex = 8; uiConfigIndex < 9; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_DEMONSTEP"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                for(let i = 0; i < groupLen; i++)
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = true;
                    }
            }
        }
    }
}

let config = [
    {
        name: "epwmDE_enableMode",
        displayName : "Enable DE Mode",
        description : 'Check to enable EPWM Diode Emulation Mode Operation',
        hidden      : false,
        default     : false,
        onChange    : onClickEnable
    },

    {
        name: "epwmDE_selectMode",
        displayName : "Select DE Mode",
        description : 'Select EPWM Diode Emulation Mode Operation',
        hidden      : true,
        default     : device_peripheral.EPWM_DiodeEmulationMode[0].name,
        options     : device_peripheral.EPWM_DiodeEmulationMode,
    },

    {
        name: "epwmDE_reEntryDelay",
        displayName : "Set DE Mode Reentry Delay",
        description : 'Set EPWM Diode Emulation Mode Reentry delay',
        hidden      : true,
        default     : 0,
    }
]

config = config.concat(
    [
        {
            name: "GROUP_DECOMPSEL",
            displayName: "DECOMPSEL",
            description: "",
            config:
            [
                {
                    name: "epwmDE_TripL",
                    displayName : "TRIPL",
                    description : 'Set DE trip source for TripL',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationTripSource[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationTripSource
                },
                {
                    name: "epwmDE_TripH",
                    displayName : "TRIPH",
                    description : 'Set DE trip source for TripH',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationTripSource[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationTripSource
                }
            ]
        }
    ]
)

config = config.concat(
    [
        {
            name: "GROUP_DEACTCTL",
            displayName: "DEACTCTL",
            description: "",
            config:
            [
                {
                    name: "epwmDE_TripSelA",
                    displayName : "Trip For Channel A",
                    description : 'Select Trip for channel A',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationTripSelect[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationTripSelect
                },
                {
                    name: "epwmDE_PWMA",
                    displayName : "PWMA Signal In DE Mode",
                    description : 'Select DE PWMA signal',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationSignal[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationSignal
                },
                {
                    name: "epwmDE_TripSelB",
                    displayName : "Trip For Channel B",
                    description : 'Select Trip for channel B',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationTripSelect[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationTripSelect
                },
                {
                    name: "epwmDE_PWMB",
                    displayName : "PWMB Signal In DE Mode",
                    description : 'Select DE PWMB signal',
                    hidden      : true,
                    default     : device_peripheral.EPWM_DiodeEmulationSignal[0].name,
                    options     : device_peripheral.EPWM_DiodeEmulationSignal
                },
                {
                    name: "epwmDE_TripEnable",
                    displayName : "Bypass DE PWM Generation Logic",
                    description : 'Bypass DE PWM generation Logic',
                    hidden      : true,
                    default     : false,
                },
            ]
        }
    ]
)

config = config.concat(
    [
        {
            name: "epwmDE_Frc",
            displayName : "Force Set DEACTIVE Flag",
            description : 'Force DEACTIVE flag to 1',
            hidden      : true,
            default     : false,
        },
        {
            name: "epwmDE_TripMonitorMode",
            displayName : "Enable Trip Monitor Mode",
            description : 'Enable Trip Monitor Mode',
            hidden      : true,
            default     : false,
            onChange    : onClickMonMode
        },
        {
            name: "epwmDE_Threshold",
            displayName : "DE Monitor Mode Threshold",
            description : 'Defines the threshold of DE monitor counter',
            hidden      : true,
            default     : 0,
        }
    ]
)

config = config.concat(
    [
        {
            name: "GROUP_DEMONSTEP",
            displayName: "DEMONSTEP",
            description: "",
            config:
            [
                {
                    name: "epwmDE_DecrementStep",
                    displayName : "Decrement Step Size",
                    description : 'Defines the decrement step of DEMONCNT.CNT counter',
                    hidden      : true,
                    default     : 0,
                },
                {
                    name: "epwmDE_IncrementStep",
                    displayName : "Increment Step Size",
                    description : 'Defines the increment step of DEMONCNT.CNT counter',
                    hidden      : true,
                    default     : 0,
                }
            ]
        }
    ]
)

let epwmDESubmodule = {
    displayName: "EPWM Diode Emulation",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_DE",
    description: "Enhanced Pulse Width Modulator Diode Emulation Mode Operation",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
}

exports = epwmDESubmodule;