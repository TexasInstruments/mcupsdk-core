
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/crc/soc/crc_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

let crc_module_name = "/drivers/crc/crc";

let crc_module = {
    displayName: "CRC",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/crc/templates/crc.h.xdt",
            moduleName: crc_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: crc_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_CRC",
    config: [
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    validate: validate,
    modules: function(instance) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    getInstanceConfig,
    getClockEnableIds,
};

exports = crc_module;
