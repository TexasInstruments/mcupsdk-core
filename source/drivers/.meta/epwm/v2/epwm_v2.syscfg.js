
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}`);

function getStaticConfigArr() {
    return system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}`).getStaticConfigArr();
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

function epwmFilter(instance,peripheral) {
    /* EPWM appears in SYSCFG by mistake - fix it by this workaround */
    let blocked_epwm =[getInterfaceName(instance)];
    let found = blocked_epwm.find(
        function(str) {
            return str == peripheral.name;
        }
    );
    return !found;
}

function pinmuxRequirements(instance) {
    let interfaceName = getInterfaceName(instance);
    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "0", "Output Channel 0");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "1", "Output Channel 1");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "SYNCO", "SYNC OUT")
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "SYNCI", "SYNC IN");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);

    let peripheral = {
        name          : interfaceName,
        displayName   : "EPWM",
        interfaceName : interfaceName,
        filter        : epwmFilter,
        resources     : resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
     return soc.getInterfaceName(instance);
}

function getPeripheralPinNames(instance) {
    return [ "0", "1", "SYNCO", "SYNCI" ];
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function validate(instance, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

let epwm_module_name = "/drivers/epwm/epwm";

let epwm_module = {
    displayName: "EPWM",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/epwm/templates/epwm.h.xdt",
            moduleName: epwm_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: epwm_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/epwm/templates/epwm_init.c.xdt",
            driver_deinit: "/drivers/epwm/templates/epwm_deinit.c.xdt",
            moduleName: epwm_module_name,
        },
    },
    defaultInstanceName: "CONFIG_EPWM",
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
};

exports = epwm_module;
