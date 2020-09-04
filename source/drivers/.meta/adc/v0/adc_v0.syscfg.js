
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

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

function pinmuxRequirements(instance) {
    let interfaceName = getInterfaceName(instance);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "AIN0", "Input Channel 1");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN1", "Input Channel 2");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN2", "Input Channel 3");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN3", "Input Channel 4");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN4", "Input Channel 5");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN5", "Input Channel 6");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN6", "Input Channel 7");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "AIN7", "Input Channel 8");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "EXT_TRIGGER0", "External Trigger 0");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "EXT_TRIGGER1", "External Trigger 1");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);

    let peripheral = {
        name          : interfaceName,
        displayName   : "ADC",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
    return "ADC";
}

function getPeripheralPinNames(instance) {
    return [ "AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "AIN7", "EXT_TRIGGER0", "EXT_TRIGGER1" ];
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function validate(instance, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

let adc_module_name = "/drivers/adc/adc";

let adc_module = {
    displayName: "ADC",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/adc/templates/adc.h.xdt",
            moduleName: adc_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: adc_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: adc_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ADC",
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
    getClockEnableIds,
    getPeripheralPinNames,
};

exports = adc_module;
