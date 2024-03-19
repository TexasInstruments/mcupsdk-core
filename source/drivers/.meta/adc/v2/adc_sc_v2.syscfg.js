let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);

let config = [
    {
        name: "GROUP_SAFECHECKCFG",
        displayName: "ADC Safety Checker Configuration",
        collapsed: false,
        config: []
    },
];

function filterHardware(component)
{
    return (common.typeMatches(component.type, ["ADC"]));
}

function moduleInstances(inst)
{
    let components = new Array();

    components.push(
        {
            name: "adc_safetycheckertile",
            group: "GROUP_SAFECHECKCFG",
            displayName: "Safety Checker Tile",
            moduleName: "/drivers/adc/v2/adcsafetycheckertile.syscfg.js",
            useArray: true,
            collapsed: true,
        }
    )

    components.push(
        {
            name: "adc_safetycheckeraggr",
            group: "GROUP_SAFECHECKCFG",
            displayName: "Safety Checker Interrupt/Event Configuration",
            moduleName: "/drivers/adc/v2/adcsafetycheckeraggr.syscfg.js",
            useArray: true,
            collapsed: true,
        }
    )

    return components;
}

var adcSCModule = {
    peripheralName: "ADCSafetyChecker",
    displayName: "ADC SAFETY CHECKER",
    defaultInstanceName: "myADCSAFETYCHECK",
    description: "ADC Safety Checker Module",
    filterHardware : filterHardware,
    moduleStatic          : {
        config          : config,
        moduleInstances : moduleInstances,
    },
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/adc/templates/adc_sc.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/adc/templates/adc_sc_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/adc/templates/adc_sc_open_close_config.c.xdt",
            driver_open: "/drivers/adc/templates/adc_sc_open.c.xdt",
        },
    },
};


exports = adcSCModule;