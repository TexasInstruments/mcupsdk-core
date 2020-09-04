
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getStaticConfigArr() {
    return system.getScript(`/drivers/eqep/soc/eqep_${common.getSocName()}`).getStaticConfigArr();
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

    pinResource = pinmux.getPinRequirements(interfaceName, "A", "Quadrature input A");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "B", "Quadrature input B");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "I", "Index input/output");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "S", "Strobe input/output");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);

    let peripheral = {
        name          : interfaceName,
        displayName   : "EQEP Instance",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
    return "EQEP";
}

function getPeripheralPinNames(instance) {
    return [ "A", "B", "I", "S" ];
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

let eqep_module_name = "/drivers/eqep/eqep";

let eqep_module = {
    displayName: "EQEP",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/eqep/templates/eqep.h.xdt",
            moduleName: eqep_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: eqep_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: eqep_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            moduleName: eqep_module_name,
        },
    },
    defaultInstanceName: "CONFIG_EQEP",
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

exports = eqep_module;
