
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getConfigArr() {
    return system.getScript(`/usb/tinyusb/tinyusb_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance,
    };
};

function pinmuxRequirements(instance) {
    let interfaceName = getInterfaceName(instance);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "DRVVBUS", "USB VBUS control output"));
    let peripheral = {
        name          : interfaceName,
        displayName   : "USB",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getInterfaceName(instance) {
    return "USB";
}

function getPeripheralPinNames(instance) {
    return [ "DRVVBUS" ];
}

let tinyusb_module_name = "/usb/tinyusb/tinyusb";

let tinyusb_module = {
    displayName: "TinyUSB",
    templates: {
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/usb/tinyusb/templates/tinyusb_open_close_config.c.xdt",
            driver_open: "/usb/tinyusb/templates/tinyusb_open.c.xdt",
            driver_close: "/usb/tinyusb/templates/tinyusb_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/usb/tinyusb/templates/tinyusb_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: tinyusb_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: tinyusb_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_TINYUSB",
    config: [

    ],
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
};

/*
 *  ======== validate ========
 */
function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

exports = tinyusb_module;
