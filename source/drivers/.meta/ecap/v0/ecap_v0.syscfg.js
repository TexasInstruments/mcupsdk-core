
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getStaticConfigArr() {
    return system.getScript(`/drivers/ecap/soc/ecap_${common.getSocName()}`).getStaticConfigArr();
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

function pinmuxRequirements(instance) {
    let interfaceName = getInterfaceName(instance);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "IN_APWM_OUT", "ECAP Capture input / PWM output");
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
    return "ECAP";
}

function getPeripheralPinNames(instance) {
    return [ "IN_APWM_OUT" ];
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
