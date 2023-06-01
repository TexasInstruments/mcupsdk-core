let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

function onChangeDeadbandMode(inst, ui, mode)
{
    if (mode == "AH")
    {
        inst.epwmDeadband_inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.epwmDeadband_inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.epwmDeadband_polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.epwmDeadband_polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.epwmDeadband_enableRED = true;
        inst.epwmDeadband_enableFED = true;

        inst.epwmDeadband_outputSwapOutA = false;
        inst.epwmDeadband_outputSwapOutB = false;
    }
    else if (mode == "AL")
    {
        inst.epwmDeadband_inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.epwmDeadband_inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.epwmDeadband_polarityRED = "EPWM_DB_POLARITY_ACTIVE_LOW";
        inst.epwmDeadband_polarityFED = "EPWM_DB_POLARITY_ACTIVE_LOW";

        inst.epwmDeadband_enableRED = true;
        inst.epwmDeadband_enableFED = true;

        inst.epwmDeadband_outputSwapOutA = false;
        inst.epwmDeadband_outputSwapOutB = false;

    }
    else if (mode == "AHC")
    {
        inst.epwmDeadband_inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.epwmDeadband_inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.epwmDeadband_polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.epwmDeadband_polarityFED = "EPWM_DB_POLARITY_ACTIVE_LOW";

        inst.epwmDeadband_enableRED = true;
        inst.epwmDeadband_enableFED = true;

        inst.epwmDeadband_outputSwapOutA = false;
        inst.epwmDeadband_outputSwapOutB = false;
    }
    else if (mode == "ALC")
    {
        inst.epwmDeadband_inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.epwmDeadband_inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.epwmDeadband_polarityRED = "EPWM_DB_POLARITY_ACTIVE_LOW";
        inst.epwmDeadband_polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.epwmDeadband_enableRED = true;
        inst.epwmDeadband_enableFED = true;

        inst.epwmDeadband_outputSwapOutA = false;
        inst.epwmDeadband_outputSwapOutB = false;
    }
    else if (mode == "DUAL")
    {
        inst.epwmDeadband_inputRED = "EPWM_DB_INPUT_EPWMB";
        inst.epwmDeadband_inputFED = "EPWM_DB_INPUT_DB_RED";

        inst.epwmDeadband_polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.epwmDeadband_polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.epwmDeadband_enableRED = false;
        inst.epwmDeadband_enableFED = true;

        inst.epwmDeadband_outputSwapOutA = false;
        inst.epwmDeadband_outputSwapOutB = false;
    }
}

function onChangeEnableDelays(inst, ui)
{
    if (inst.epwmDeadband_enableRED == true)
    {
       ui.epwmDeadband_delayRED.hidden = false;
    }
    else if (inst.epwmDeadband_enableRED == false)
    {
       ui.epwmDeadband_delayRED.hidden = true;
       inst.epwmDeadband_delayRED = 0;// Set to default
    }
    if (inst.epwmDeadband_enableFED == true)
    {
       ui.epwmDeadband_delayFED.hidden = false;
    }
    else if (inst.epwmDeadband_enableFED == false)
    {
       ui.epwmDeadband_delayFED.hidden = true;
       inst.epwmDeadband_delayFED = 0; // Set to default
    }
}

function onChangeShadowModes(inst, ui)
{
    if (!inst["epwmDeadband_controlShadowMode"])
    {
        ui["epwmDeadband_controlShadowLoadEvent"].hidden = true;
        inst.epwmDeadband_controlShadowLoadEvent= device_peripheral.EPWM_DeadBandControlLoadMode[0].name;
    }
    else
    {
        ui["epwmDeadband_controlShadowLoadEvent"].hidden = false;
    }
    if (!inst["epwmDeadband_redShadowMode"])
    {
        ui["epwmDeadband_redShadowLoadEvent"].hidden = true;
        inst.epwmDeadband_redShadowLoadEvent = device_peripheral.EPWM_RisingEdgeDelayLoadMode[0].name
    }
    else
    {
        ui["epwmDeadband_redShadowLoadEvent"].hidden = false;
    }
    if (!inst["epwmDeadband_fedShadowMode"])
    {
        ui["epwmDeadband_fedShadowLoadEvent"].hidden = true;
        inst.epwmDeadband_fedShadowLoadEvent = device_peripheral.EPWM_FallingEdgeDelayLoadMode[0].name
    }
    else
    {
         ui["epwmDeadband_fedShadowLoadEvent"].hidden = false;
    }
}

/* Array of configurables that are common across device families */
let config = [
    {
        name: "epwmDeadband_inputRED",
        displayName : "Rising Edge Delay Input",
        description : 'Select the source for DBRED (Rising Edge Delay)',
        hidden      : false,
        default     : device_peripheral.EPWM_DB_INPUT[0].name,
        options     : device_peripheral.EPWM_DB_INPUT.slice(0,2)
    },
    {
        name: "epwmDeadband_inputFED",
        displayName : "Falling Edge Delay Input",
        description : 'Select the source for DBFED (Falling Edge Delay)',
        hidden      : false,
        default     : device_peripheral.EPWM_DB_INPUT[0].name,
        options     : device_peripheral.EPWM_DB_INPUT
    },

    {
        name: "epwmDeadband_polarityRED",
        displayName : "Rising Edge Delay Polarity",
        description : 'Whether or not to invert RED (Rising Edge Delay)',
        hidden      : false,
        default     : device_peripheral.EPWM_DeadBandPolarity[0].name,
        options     : device_peripheral.EPWM_DeadBandPolarity
    },

    {
        name: "epwmDeadband_polarityFED",
        displayName : "Falling Edge Delay Polarity",
        description : 'Whether or not to invert FED (Falling Edge Delay)',
        hidden      : false,
        default     : device_peripheral.EPWM_DeadBandPolarity[0].name,
        options     : device_peripheral.EPWM_DeadBandPolarity
    },

    {
        name: "epwmDeadband_enableRED",
        displayName : "Enable Rising Edge Delay",
        description : 'Enable DBRED (Rising Edge Delay) by choosing it as the source for the A path',
        hidden      : false,
        default     : false,
        onChange    : onChangeEnableDelays
    },

    {
        name: "epwmDeadband_delayRED",
        displayName : "Rising Edge Delay Value",
        description : 'DBRED (Rising Edge Delay) delay value',
        hidden      : true,
        default     : 0,
    },

    {
        name: "epwmDeadband_enableFED",
        displayName : "Enable Falling Edge Delay",
        description : 'Enable DBFED (Falling Edge Delay) by choosing it as the source for the B path',
        hidden      : false,
        default     : false,
        onChange    : onChangeEnableDelays
    },

    {
        name: "epwmDeadband_delayFED",
        displayName : "Falling Edge Delay Value",
        description : 'DBFED (Falling Edge Delay) delay value',
        hidden      : true,
        default     : 0,
    },

    {
        name: "epwmDeadband_outputSwapOutA",
        displayName : "Swap Output for EPWMxA",
        description : 'Check to enable output swap. Channel A path to Out B.',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmDeadband_outputSwapOutB",
        displayName : "Swap Output for EPWMxB",
        description : 'Check to enable output swap. Channel B path to Out A.',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmDeadband_dbControlGld",
        displayName : "Enable Deadband Control Global Load",
        description : 'Use global load configuration for deadband control',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmDeadband_controlShadowMode",
        displayName : "Enable Deadband Control Shadow Mode",
        description : 'Enable shadowing of the DBCTL (Deadband Control) register.',
        hidden      : false,
        default     : false,
        onChange    : onChangeShadowModes,
    },
    {
        name: "epwmDeadband_controlShadowLoadEvent",
        displayName : "Deadband Control Shadow Load Event",
        description : 'Shadow to active load event for the DBCTL (Deadband Control) register.',
        hidden      : true,
        default     : device_peripheral.EPWM_DeadBandControlLoadMode[0].name,
        options     : device_peripheral.EPWM_DeadBandControlLoadMode
    },
    {
        name: "epwmDeadband_redGld",
        displayName : "Enable RED Global Load",
        description : 'Use global load configuration for RED',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmDeadband_redShadowMode",
        displayName : "Enable RED Shadow Mode",
        description : 'Enable shadowing of the RED (Rising Edge Delay) register.',
        hidden      : false,
        default     : false,
        onChange    : onChangeShadowModes,
    },
    {
        name: "epwmDeadband_redShadowLoadEvent",
        displayName : "RED Shadow Load Event",
        description : 'Shadow to active load event for the RED (Rising Edge Delay) register.',
        hidden      : true,
        default     : device_peripheral.EPWM_RisingEdgeDelayLoadMode[0].name,
        options     : device_peripheral.EPWM_RisingEdgeDelayLoadMode
    },
    {
        name: "epwmDeadband_fedGld",
        displayName : "Enable FED Global Load",
        description : 'Use global load configuration for FED',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmDeadband_fedShadowMode",
        displayName : "Enable FED Shadow Mode",
        description : 'Enable shadowing of the FED (Falling Edge Delay) register.',
        hidden      : false,
        default     : false,
        onChange    : onChangeShadowModes,
    },
    {
        name: "epwmDeadband_fedShadowLoadEvent",
        displayName : "FED Shadow Load Event",
        description : 'Shadow to active load event for the FED (Falling Edge Delay) register.',
        hidden      : true,
        default     : device_peripheral.EPWM_FallingEdgeDelayLoadMode[0].name,
        options     : device_peripheral.EPWM_FallingEdgeDelayLoadMode
    },
    {
        name: "epwmDeadband_deadbandCounterClockRate",
        displayName : "Dead Band Counter Clock Rate",
        description : 'Dead band counter clock rate.',
        hidden      : false,
        default     : device_peripheral.EPWM_DeadBandClockMode[0].name,
        options     : device_peripheral.EPWM_DeadBandClockMode
    },

];



let epwmDeadbandSubmodule = {
    displayName: "EPWM Dead-Band",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_DB",
    description: "Enhanced Pulse Width Modulator Dead-Band",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmDeadbandSubmodule;