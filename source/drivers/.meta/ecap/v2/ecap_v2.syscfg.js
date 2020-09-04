
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/ecap/soc/ecap_${common.getSocName()}`);

function getStaticConfigArr() {
    return soc.getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getStaticConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);
     return {
        ...config,
        ...moduleInstance,
     };
};

function pinmuxRequirements(instance) {
    let interfaceName = getInterfaceName(instance);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "PWMO", "ECAP Capture input / PWM output");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);

    let peripheral = {
        name          : interfaceName,
        displayName   : "ECAP Instance",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
    return "RCSS_ECAP";
}

function getPeripheralPinNames(instance) {
    return [ "PWMO" ];
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(instance) {

    let instConfig = getInstanceConfig(instance);

    return instConfig.clockFrequencies;
}

function validate(instance, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

let ecap_module_name = "/drivers/ecap/ecap";

let ecap_module = {
    displayName: "ECAP",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/ecap/templates/ecap.h.xdt",
            moduleName: ecap_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: ecap_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
           moduleName: ecap_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            moduleName: ecap_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ECAP",
   
    config: [
       common.ui.makeInstanceConfig(getStaticConfigArr()),
    ],
    validate: validate,
    modules: function(instance) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
};

exports = ecap_module;
