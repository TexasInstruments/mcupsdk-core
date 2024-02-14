let common   = system.getScript("/common");
let device_driverlib_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

let TzEventConfigNames = [];
let TzAdvEventConfigNames = [];

function onChangeUseAdvancedEPWMTripZoneActions(inst, ui)
{
    for (let tzEventConfigNameIndex in TzEventConfigNames)
    {
        let tzEventConfigName = TzEventConfigNames[tzEventConfigNameIndex];
        if (inst.epwmTripZone_useAdvancedEPWMTripZoneActions)
        {
            ui[tzEventConfigName].hidden = true;
            inst[tzEventConfigName] = device_driverlib_peripheral.EPWM_TripZoneAction[0].name; // Set to default
        }
        else
        {
            ui[tzEventConfigName].hidden = false;
        }
    }
    for (let tzAdvEventConfigNameIndex in TzAdvEventConfigNames)
    {
        let tzAdvEventConfigName = TzAdvEventConfigNames[tzAdvEventConfigNameIndex];
        if (!inst.epwmTripZone_useAdvancedEPWMTripZoneActions)
        {
            ui[tzAdvEventConfigName].hidden = true;
            inst[tzAdvEventConfigName] = device_driverlib_peripheral.EPWM_TripZoneAdvancedAction[0].name; // Set to default
        }
        else
        {
            ui[tzAdvEventConfigName].hidden = false;
        }

    }

}

/* Array of configurables that are common across device families */
let config = [
    {
        name: "epwmTripZone_useAdvancedEPWMTripZoneActions",
        displayName : "Use Advanced EPWM Trip Zone Actions",
        description : 'Check this to use Advanced EPWM trip zone actions. Uncheck to use legacy (TZCTL2[ETZE])',
        hidden      : false,
        default     : false,
        onChange    : onChangeUseAdvancedEPWMTripZoneActions
    },

];

for (let tzEventIndex in device_driverlib_peripheral.EPWM_TripZoneEvent)
{
    let tzEvent = device_driverlib_peripheral.EPWM_TripZoneEvent[tzEventIndex];
    let tzEventName = tzEvent.name.replace("EPWM_TZ_ACTION_EVENT_", "");
    let tzEventConfig = {
        name: "epwmTripZone_" + tzEvent.name,
        displayName : tzEventName + " Event",
        description : 'The action to take on ' + tzEvent.name + ". " + "(" + tzEvent.displayName + ")",
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_TripZoneAction[0].name,
        options     : device_driverlib_peripheral.EPWM_TripZoneAction
    }
    config.push(tzEventConfig);
    TzEventConfigNames.push(tzEventConfig.name);
}


for (let tzEventIndex in device_driverlib_peripheral.EPWM_TripZoneAdvancedEvent)
{
    let tzEvent = device_driverlib_peripheral.EPWM_TripZoneAdvancedEvent[tzEventIndex];
    let tzEventName = tzEvent.name.replace("EPWM_TZ_ADV_ACTION_EVENT_", "");
    let tzEventConfig = {
        name: "epwmTripZone_" + tzEvent.name,
        displayName : tzEventName + " Event (Adv)",
        description : 'The action to take on ' + tzEvent.name + ". " + "(" + tzEvent.displayName + ")",
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction[0].name,
        options     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction
    }
    config.push(tzEventConfig);
    TzAdvEventConfigNames.push(tzEventConfig.name);
}

for (let epwmX of ["A", "B"])
{
    for (let tzEventIndex in device_driverlib_peripheral.EPWM_TripZoneAdvDigitalCompareEvent)
    {
        let tzEvent = device_driverlib_peripheral.EPWM_TripZoneAdvDigitalCompareEvent[tzEventIndex];
        let tzEventName = tzEvent.name.replace("EPWM_TZ_ADV_ACTION_EVENT_", "").replace("DCx", "DC" + epwmX);
        let tzEventConfig = {
            name: "epwmTripZone_" + tzEvent.name + "_" + epwmX,
            displayName : tzEventName + " Event (Adv)",
            description : 'For EPWMx' + epwmX + ' ,the action to take on ' + tzEvent.name + ". " + "(" + tzEvent.displayName + ")",
            hidden      : true,
            default     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction[0].name,
            options     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction
        }
        config.push(tzEventConfig);
        TzAdvEventConfigNames.push(tzEventConfig.name);
    }
}

let oneShotConfig = [
    {
        name: "epwmTripZone_oneShotSource",
        displayName : "One-Shot Source",
        description : 'Check to enable the source to the One-Shot OR gate',
        hidden      : false,
        minSelections : 0,
        default     : [],
        options     : device_driverlib_peripheral.EPWM_TZ_SIGNAL.slice(8,16)
    },
    {
        name: "epwmTripZone_oneShotSourceAdditional",
        displayName : "Additional One-Shot Source",
        description : 'Check to enable the source to the One-Shot OR gate',
        hidden      : false,
        minSelections : 0,
        default     : [],
        options     : device_driverlib_peripheral.EPWM_TZ_SIGNAL.slice(17,18)
    },

];

let cbcConfig = [
    {
        name: "epwmTripZone_cbcSource",
        displayName : "CBC Source",
        description : 'Check to enable the source to the CBC OR gate',
        hidden      : false,
        minSelections : 0,
        default     : [],
        options     : device_driverlib_peripheral.EPWM_TZ_SIGNAL.slice(0,8)
    },
    {
        name: "epwmTripZone_cbcSourceAdditional",
        displayName : "Additional CBC Source",
        description : 'Check to enable the source to the CBC OR gate',
        hidden      : false,
        minSelections : 0,
        default     : [],
        options     : device_driverlib_peripheral.EPWM_TZ_SIGNAL.slice(16,17)
    },
    {
        name: "epwmTripZone_cbcPulse",
        displayName : "CBC Latch Clear Signal",
        description : 'Select the CBC Trip Zone Latch Clear Signal (TZCLR[CBCPULSE])',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_CycleByCycleTripZoneClearMode[0].name,
        options     : device_driverlib_peripheral.EPWM_CycleByCycleTripZoneClearMode
    },

];

let interruptConfig = [
    {
        name: "epwmTripZone_tzInterruptSource",
        displayName : "TZ Interrupt Source (ORed)",
        description : 'Select the all the signals which should be enabled to generate a TZINT.',
        hidden      : false,
        minSelections : 0,
        default     : [],
        options     : device_driverlib_peripheral.EPWM_TZ_INTERRUPT
    },
]

config = config.concat(oneShotConfig);
config = config.concat(cbcConfig);
config = config.concat(interruptConfig);


let epwmTripZoneSubmodule = {
    displayName: "EPWM Trip Zone",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_TZ",
    description: "Enhanced Pulse Width Modulator Trip Zone",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmTripZoneSubmodule;