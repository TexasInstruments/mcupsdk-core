let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);

let module = system.getScript("adc_v2");

function getStaticConfigArr() {
    return system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceNameAdcSC(inst);
}

// let Common   = system.getScript("/driverlib/Common.js");
// let InternalConnections = system.getScript("/driverlib/adc/adc_internalConnections.js")
// let device_driverlib_peripheral =
//     system.getScript("/driverlib/device_driverlib_peripherals/" +
//         Common.getDeviceName().toLowerCase() + "_adc.js");
// let device_driverlib_memmap =
//     system.getScript("/driverlib/device_driverlib_peripherals/" +
//         Common.getDeviceName().toLowerCase() + "_memmap.js");

var ADC_SFTYCH_SOURCE =[];

let sftychkrTotal = 12

for (var sftychkr in device_peripheral.ADC_Checker)
{
    for (var chkrslt in device_peripheral.ADC_SafetyCheckResult)
    {
        //var currentchkrslt= device_driverlib_memmap.ADCSAFETYCHKMemoryMap[sftychkr].name.replace("_BASE","").replace("ADC","")
        //var currentsftychkr = device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_","")

        ADC_SFTYCH_SOURCE.push(
            {name: device_peripheral.ADC_Checker[sftychkr].name.replace("ADC_","") + device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_",""),
            displayName: device_peripheral.ADC_Checker[sftychkr].name.replace("ADC_","") + " "+ device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_","")
        },
        )
    }

}

function onChangeSafetyCheckerEVNTs(inst, ui){
    for(var evntIndex in device_peripheral.ADC_SafetyCheckEvent)
    {
        var currentEvnt = device_peripheral.ADC_SafetyCheckEvent[evntIndex].name
        var evnti = (currentEvnt).replace(/[^0-9]/g,'')
        var sftyEVNTConfigs =
            [
                "chkEvent" + evnti.toString() + "Source",
            ]

        if (ui)
        {
            for (var sftyConfig of sftyEVNTConfigs)
            {
                if(inst["enableEvent"].includes(currentEvnt))
                {
                    ui[sftyConfig].hidden = false;
                }
                else
                {
                    ui[sftyConfig].hidden = true;
                }
            }
        }
    }
}
function onChangeSafetyCheckerINTs(inst, ui) {
    if (ui) {
        if((inst.enableInterrupt)) {
            ui["eventInterruptSource"].hidden = false;
        }
        else {
            ui["eventInterruptSource"].hidden = true;
        }
    }
}

let config = [
    {
        name: "GROUP_ADC_CHECKER_TILES",
        config: [],
    }
];
var sftyINT_configs =[];
sftyINT_configs = sftyINT_configs.concat([
    {
        name            : "enableInterrupt",
        displayName     : "Enable Interrupt",
        description     : 'Each ADC result safety checker tile can generate an interrupt signal from out-of-tolerance (OOT) flag and result overflow flags OVF1 and OVF2',
        longDescription : 'Each ADC result safety checker tile generates an interrupt pulse when an enabled out-of-tolerance (OOT) or overflow OVF1 or OVF2 event occurs. No further interrupt pulses will be generated until the interrupt flag is cleared.',
        hidden          : false,
        default         : false,
        onChange        : onChangeSafetyCheckerINTs,
    },
    {
        name            : "eventInterruptSource",
        displayName     : "Select source for interrupt",
        description     : 'Each ADC result safety checker tile can generate an interrupt signal from out-of-tolerance (OOT) flag and result overflow flags OVF1 and OVF2',
        hidden          : true,
        minSelections   : 0,
        default         : [],
        options         : ADC_SFTYCH_SOURCE,
        onChange        : onChangeSafetyCheckerINTs,
    },
    {
        name            : "enableEvent",
        displayName     : "Enable Event",
        description     : 'safety checker tiles can generate events that can be sent to the X-BAR, so that automatic hardware actions such as an ePWM trip can be generated',
        longDescription : '',
        hidden          : false,
        minSelections   : 0,
        default         : [],
        options         :device_peripheral.ADC_SafetyCheckEvent,
        onChange        : onChangeSafetyCheckerEVNTs,
    },

])
for(var evntIndex in device_peripheral.ADC_SafetyCheckEvent)
    {
        var currentEvnt = device_peripheral.ADC_SafetyCheckEvent[evntIndex].name
        var evnti = (currentEvnt).replace(/[^0-9]/g,'')
        var sfty_Configs = [
        {
            name: "chkEvent" + evnti.toString() + "Source",
            displayName : "Select Source",
            description : 'Select Source for this event',
            hidden      : true,
            minSelections   : 0,
            default         : [],
            options     : ADC_SFTYCH_SOURCE,
        },

    ]
    sftyINT_configs= sftyINT_configs.concat ([{
        name: "GROUP_EVENT" +evnti.toString(),
        displayName: "Safety Check Event " + evnti.toString(),
        description: "",
        longDescription: "",
        config: sfty_Configs
    }])
    }

config= config.concat ([
    {
        name: "GROUP_INT",
        displayName: "Interrupt/Event Configuration",
        description: "Interrupt/Event Configuration",
        longDescription: "",
        config: sftyINT_configs
    },
])

function onValidateStatic(mod, stat)
{
    // console.log("stat");
    // Common.printDebugObject(stat);
    // console.log("mod");
    // Common.printDebugObject(mod);
}

function filterHardware(component)
{
    return (Common.typeMatches(component.type, ["ADC"]));
}


function onValidate(inst, validation) {
}

var adcscModule = {
    peripheralName: "ADCSafetyChecker",
    displayName: "ADC SAFETY CHECKER",
    maxInstances: 1,
    defaultInstanceName: "CONFIG_ADC_SC",
    description: "ADC Safety Checker Module",
    //longDescription: (Common.getCollateralFindabilityList("ADC")),
    filterHardware : filterHardware,

    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/adc/templates/adc_sc.h.xdt",
        },
        // "/drivers/system/system_config.c.xdt": {
        //     driver_init: "/drivers/adc/templates/adc_sc_init.c.xdt",
        // },
        // "/drivers/system/drivers_open_close.h.xdt": {
        //     driver_open_close_config: "/drivers/adc/templates/adc_sc_open_close_config.h.xdt",
        // },
        // "/drivers/system/drivers_open_close.c.xdt": {
        //     driver_open_close_config: "/drivers/adc/templates/adc_sc_open_close_config.c.xdt",
        //     driver_open: "/drivers/adc/templates/adc_sc_open.c.xdt",
        // },
        // boardc : "/driverlib/adc/adcsafetychecker.board.c.xdt",
        // boardh : "/driverlib/adc/adcsafetychecker.board.h.xdt"
    },

    moduleStatic: {
        name: "adcSafety",
        displayName: "ADC Safety Checker",
        config: config,
        getInstanceConfig: getInstanceConfig,
        getInterfaceName: getInterfaceName,
        moduleInstances: (inst) =>
        {
            return [{
            name: "safetyCheckerTiles",
            displayName: "Safety Checker Tiles",
            moduleName: "/drivers/.meta/adc/v2/adc_sc_tile_v2.syscfg.js",
            useArray: true,
            collapsed: false,
            group: "GROUP_ADC_CHECKER_TILES",
            }]
        }
    },
    validate    : onValidate,
};

exports = adcscModule;