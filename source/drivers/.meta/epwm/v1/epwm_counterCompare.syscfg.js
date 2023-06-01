let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

function onChangeCMPXShadowLoadMode(inst, ui)
{
    // CMPA
    if (inst.epwmCounterCompare_enableShadowLoadModeCMPA == true)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPA.hidden = false;
    }
    else if (inst.epwmCounterCompare_enableShadowLoadModeCMPA == false)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPA.hidden = true;
        inst.epwmCounterCompare_shadowLoadModeCMPA = device_peripheral.EPWM_CounterCompareLoadMode[0].name;
    }

    // CMPB
    if (inst.epwmCounterCompare_enableShadowLoadModeCMPB == true)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPB.hidden = false;
    }
    else if (inst.epwmCounterCompare_enableShadowLoadModeCMPB == false)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPB.hidden = true;
        inst.epwmCounterCompare_shadowLoadModeCMPB = device_peripheral.EPWM_CounterCompareLoadMode[0].name;
    }

    // CMPC
    if (inst.epwmCounterCompare_enableShadowLoadModeCMPC == true)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPC.hidden = false;
    }
    else if (inst.epwmCounterCompare_enableShadowLoadModeCMPC == false)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPC.hidden = true;
        inst.epwmCounterCompare_shadowLoadModeCMPC = device_peripheral.EPWM_CounterCompareLoadMode[0].name;
    }

    // CMPD
    if (inst.epwmCounterCompare_enableShadowLoadModeCMPD == true)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPD.hidden = false;
    }
    else if (inst.epwmCounterCompare_enableShadowLoadModeCMPD == false)
    {
        ui.epwmCounterCompare_shadowLoadModeCMPD.hidden = true;
        inst.epwmCounterCompare_shadowLoadModeCMPD = device_peripheral.EPWM_CounterCompareLoadMode[0].name;
    }

}

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "GROUP_CMPA",
        displayName: "CMPA",
        description: "",
        longDescription: "",
        config: [
            {
                name: "epwmCounterCompare_cmpA",
                displayName : "Counter Compare A (CMPA)",
                description : 'Counter Compare A (CMPA) value',
                hidden      : false,
                default     : 0,
            },
            {
                name: "epwmCounterCompare_cmpAGld",
                displayName : "Enable Counter Compare A Global Load",
                description : 'Use global load configuration for CMPA',
                hidden      : false,
                default     : false,
            },
            {
                name: "epwmCounterCompare_enableShadowLoadModeCMPA",
                displayName : "Enable Shadow Counter Compare A",
                description : 'Enable Shadow Counter Compare A (CMPA) value',
                hidden      : false,
                default     : true,
                onChange    : onChangeCMPXShadowLoadMode
            },
            {
                name: "epwmCounterCompare_shadowLoadModeCMPA",
                displayName : "Counter Compare A Shadow Load Event",
                description : 'Counter Compare A Shadow Load Event',
                hidden      : false,
                default     : device_peripheral.EPWM_CounterCompareLoadMode[0].name,
                options     : device_peripheral.EPWM_CounterCompareLoadMode,
            },
            {
                name: "epwmCounterCompare_cmpALink",
                displayName : "Counter Compare A (CMPA) Link",
                description : 'Simultaneous write to the current ePWM module CMPA register',
                hidden      : false,
                default     : device_peripheral.EPWM_CurrentLink[0].name,
                options     : device_peripheral.EPWM_CurrentLink,
            }
        ]
    },
    {
        name: "GROUP_CMPB",
        displayName: "CMPB",
        description: "",
        longDescription: "",
        config: [
            {
                name: "epwmCounterCompare_cmpB",
                displayName : "Counter Compare B (CMPB)",
                description : 'Counter Compare B (CMPB) value',
                hidden      : false,
                default     : 0,
            },
            {
                name: "epwmCounterCompare_cmpBGld",
                displayName : "Enable Counter Compare B Global Load",
                description : 'Use global load configuration for CMPB',
                hidden      : false,
                default     : false,
            },
            {
                name: "epwmCounterCompare_enableShadowLoadModeCMPB",
                displayName : "Enable Shadow Counter Compare B",
                description : 'Enable Shadow Counter Compare B (CMPB) value',
                hidden      : false,
                default     : true,
                onChange    : onChangeCMPXShadowLoadMode
            },
            {
                name: "epwmCounterCompare_shadowLoadModeCMPB",
                displayName : "Counter Compare B Shadow Load Event",
                description : 'Counter Compare B Shadow Load Event',
                hidden      : false,
                default     : device_peripheral.EPWM_CounterCompareLoadMode[0].name,
                options     : device_peripheral.EPWM_CounterCompareLoadMode,
            },
            {
                name: "epwmCounterCompare_cmpBLink",
                displayName : "Counter Compare A (CMPB) Link",
                description : 'Simultaneous write to the current ePWM module CMPB register',
                hidden      : false,
                default     : device_peripheral.EPWM_CurrentLink[0].name,
                options     : device_peripheral.EPWM_CurrentLink,
            }
        ]
    },
    {
        name: "GROUP_CMPC",
        displayName: "CMPC",
        description: "",
        longDescription: "",
        config: [
            {
                name: "epwmCounterCompare_cmpC",
                displayName : "Counter Compare C (CMPC)",
                description : 'Counter Compare C (CMPC) value',
                hidden      : false,
                default     : 0,
            },
            {
                name: "epwmCounterCompare_cmpCGld",
                displayName : "Enable Counter Compare C Global Load",
                description : 'Use global load configuration for CMPC',
                hidden      : false,
                default     : false,
            },
            {
                name: "epwmCounterCompare_enableShadowLoadModeCMPC",
                displayName : "Enable Shadow Counter Compare C",
                description : 'Enable Shadow Counter Compare C (CMPC) value',
                hidden      : false,
                default     : true,
                onChange    : onChangeCMPXShadowLoadMode
            },
            {
                name: "epwmCounterCompare_shadowLoadModeCMPC",
                displayName : "Counter Compare C Shadow Load Event",
                description : 'Counter Compare C Shadow Load Event',
                hidden      : false,
                default     : device_peripheral.EPWM_CounterCompareLoadMode[0].name,
                options     : device_peripheral.EPWM_CounterCompareLoadMode,
            },
            {
                name: "epwmCounterCompare_cmpCLink",
                displayName : "Counter Compare C (CMPC) Link",
                description : 'Simultaneous write to the current ePWM module CMPC register',
                hidden      : false,
                default     : device_peripheral.EPWM_CurrentLink[0].name,
                options     : device_peripheral.EPWM_CurrentLink,
            }
        ]
    },
    {
        name: "GROUP_CMPD",
        displayName: "CMPD",
        description: "",
        longDescription: "",
        config: [
            {
                name: "epwmCounterCompare_cmpD",
                displayName : "Counter Compare D (CMPD)",
                description : 'Counter Compare D (CMPD) value',
                hidden      : false,
                default     : 0,
            },
            {
                name: "epwmCounterCompare_cmpDGld",
                displayName : "Enable Counter Compare D Global Load",
                description : 'Use global load configuration for CMPD',
                hidden      : false,
                default     : false,
            },
            {
                name: "epwmCounterCompare_enableShadowLoadModeCMPD",
                displayName : "Enable Shadow Counter Compare D",
                description : 'Enable Shadow Counter Compare D (CMPD) value',
                hidden      : false,
                default     : true,
                onChange    : onChangeCMPXShadowLoadMode
            },
            {
                name: "epwmCounterCompare_shadowLoadModeCMPD",
                displayName : "Counter Compare D Shadow Load Event",
                description : 'Counter Compare D Shadow Load Event',
                hidden      : false,
                default     : device_peripheral.EPWM_CounterCompareLoadMode[0].name,
                options     : device_peripheral.EPWM_CounterCompareLoadMode,
            },
            {
                name: "epwmCounterCompare_cmpDLink",
                displayName : "Counter Compare D (CMPD) Link",
                description : 'Simultaneous write to the current ePWM module CMPD register',
                hidden      : false,
                default     : device_peripheral.EPWM_CurrentLink[0].name,
                options     : device_peripheral.EPWM_CurrentLink,
            }
        ]
    },
];



let epwmCCSubmodule = {
    displayName: "EPWM Counter Compare",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_CC",
    description: "Enhanced Pulse Width Modulator Counter Compare",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmCCSubmodule;