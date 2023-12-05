let common   = system.getScript("/common");
let device_driverlib_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);


function onChangeEnableDisable(inst, ui)
{
    if (inst.epwmTimebase_phaseEnable == true)
    {
        ui.epwmTimebase_phaseShift.hidden = false;
    }
    else if (inst.epwmTimebase_phaseEnable == false)
    {
        ui.epwmTimebase_phaseShift.hidden = true;
        inst.epwmTimebase_phaseShift = 0; // Set default
    }

    if (inst.epwmTimebase_periodLoadMode == "EPWM_PERIOD_SHADOW_LOAD")
    {
        ui.epwmTimebase_periodLoadEvent.hidden = false;
    }
    else if (inst.epwmTimebase_periodLoadMode == "EPWM_PERIOD_DIRECT_LOAD")
    {
        ui.epwmTimebase_periodLoadEvent.hidden = true;
        inst.epwmTimebase_periodLoadEvent = device_driverlib_peripheral.EPWM_PeriodShadowLoadMode[0].name; // Set default
    }

    if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP_DOWN")
    {
        ui.epwmTimebase_counterModeAfterSync.hidden = false;
    }
    else if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP" ||
        inst.epwmTimebase_counterMode =="EPWM_COUNTER_MODE_DOWN" ||
        inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_STOP_FREEZE")
    {
        ui.epwmTimebase_counterModeAfterSync.hidden = true;
        inst.epwmTimebase_counterModeAfterSync = device_driverlib_peripheral.EPWM_SyncCountMode[0].name;
    }
}

let config = [
    {
        name        : "epwmTimebase_emulationMode",
        displayName : "Emulation Mode",
        description : 'Behavior of the ePWM time-base counter during emulation events',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_EmulationMode[0].name,
        options     : device_driverlib_peripheral.EPWM_EmulationMode
    },
    {
        name: "epwmTimebase_clockDiv",
        displayName : "Time Base Clock Divider",
        description : 'CLKDIV: These bits select the time base clock pre-scale value, TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_ClockDivider[0].name,
        options     : device_driverlib_peripheral.EPWM_ClockDivider,
    },
    {
        name: "epwmTimebase_hsClockDiv",
        displayName : "High Speed Clock Divider",
        description : 'HSPCLKDIV: These bits determine part of the time-base clock pre-scale value, TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_HSClockDivider[0].name,
        options     : device_driverlib_peripheral.EPWM_HSClockDivider,
    },
    {
        name: "epwmTimebase_period",
        displayName : "Time Base Period",
        description : 'Period for the Time Base Counter Submodule',
        hidden      : false,
        default     : 0,
    },
    {
        name: "epwmTimebase_periodLink",
        displayName : "Time Base Period Link",
        description : 'Simultaneous write to the current ePWM module PRD register',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_CurrentLink[0].name,
        options     : device_driverlib_peripheral.EPWM_CurrentLink,
    },
    {
        name: "epwmTimebase_periodGld",
        displayName : "Enable Time Base Period Global Load",
        description : 'Use global load configuration for PRD',
        hidden      : false,
        default     : false,
    },
    {
        name: "epwmTimebase_periodLoadMode",
        displayName : "Time Base Period Load Mode",
        description : 'Period load mode for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_PeriodLoadMode[0].name,
        options     : device_driverlib_peripheral.EPWM_PeriodLoadMode,
        onChange    : onChangeEnableDisable
    },
    {
        name: "epwmTimebase_periodLoadEvent",
        displayName : "Time Base Period Load Event",
        description : 'Period load event for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_PeriodShadowLoadMode[0].name,
        options     : device_driverlib_peripheral.EPWM_PeriodShadowLoadMode,
    },
    {
        name: "epwmTimebase_counterValue",
        displayName : "Initial Counter Value",
        description : 'Initial Counter value for the Time Base Counter Submodule',
        hidden      : false,
        default     : 0,
    },
    {
        name: "epwmTimebase_counterMode",
        displayName : "Counter Mode",
        description : 'Mode of the Counter value for the Time Base Counter Submodule',
        hidden      : false,
        default     : "EPWM_COUNTER_MODE_STOP_FREEZE",
        options     : device_driverlib_peripheral.EPWM_TimeBaseCountMode,
        onChange    : onChangeEnableDisable
    },
    {
        name: "epwmTimebase_counterModeAfterSync",
        displayName : 'Counter Mode After Sync',
        description : 'The direction the time-base counter (TBCTR) will count after a synchronization event occurs',
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_SyncCountMode[0].name,
        options     : device_driverlib_peripheral.EPWM_SyncCountMode,
    },
    {
        name: "epwmTimebase_phaseEnable",
        displayName : "Enable Phase Shift Load",
        description : 'Enable phase shift load for the Time Base Counter Submodule',
        hidden      : false,
        default     : false,
        onChange    : onChangeEnableDisable
    },
    {
        name: "epwmTimebase_phaseShift",
        displayName : 'Phase Shift Value',
        description : 'Phase Shift Value for the Time Base Counter Submodule',
        hidden      : true,
        default     : 0,
    },
    {
        name: "epwmTimebase_forceSyncPulse",
        displayName : 'Force A Sync Pulse',
        description : 'Force a sync pulse for the Time Base Counter Submodule',
        hidden      : false,
        default     : false,
    },
];

config.splice.apply(config, [config.length - 2, 0].concat([
    {
        name: "epwmTimebase_syncInPulseSource",
        displayName : "Sync In Pulse Source",
        description : 'Sync in Pulse for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_SyncInPulseSource[0].name,
        options     : device_driverlib_peripheral.EPWM_SyncInPulseSource,
    },
    {
        name: "epwmTimebase_syncOutPulseMode",
        displayName : "Sync Out Pulse",
        description : 'Sync Out Pulse for the Time Base Counter Submodule',
        hidden      : false,
        minSelections: 0,
        default     : [],
        options     : [
            { name: "EPWM_SYNC_OUT_PULSE_ON_SOFTWARE", displayName: "Software force generated EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO", displayName: "Counter zero event generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B", displayName: "Counter equal to CMPB event generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C", displayName: "Counter equal to CMPC event generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D", displayName: "Counter equal to CMPD event generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC", displayName: "DCA Event 1 Sync signal generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC", displayName: "DCB Event 1 Sync signal generates EPWM sync-out pulse" },
            { name: "EPWM_SYNC_OUT_PULSE_ON_ALL", displayName: "Enable all the above sources" },
        ],
    },
    {
        name: "epwmTimebase_oneShotSyncOutTrigger",
        displayName : "One-Shot Sync Out Trigger",
        description : 'One-Shot Sync Out Trigger for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_OneShotSyncOutTrigger[0].name,
        options     : device_driverlib_peripheral.EPWM_OneShotSyncOutTrigger,
    },
    {
        name: "hrpwm_syncSource",
        displayName : "PWMSYNC Source Select",
        description : 'Selects the source of the EPWMSYNCPER signal that goes to the CMPSS and GPDAC',
        hidden      : false,
        default     : device_driverlib_peripheral.HRPWM_SyncPulseSource[0].name,
        options     : device_driverlib_peripheral.HRPWM_SyncPulseSource
    },

]));



let epwmTimebaseSubmodule = {
    displayName: "EPWM Time Base",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_TB",
    description: "Enhanced Pulse Width Modulator Time Base Counter",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmTimebaseSubmodule;