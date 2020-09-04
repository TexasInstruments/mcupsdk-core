
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/cbuff/soc/cbuff_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

let cbuff_module_name = "/drivers/cbuff/cbuff";

let cbuff_module = {
    displayName: "CBUFF",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/cbuff/templates/cbuff_config_v1.c.xdt",
            moduleName: cbuff_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/cbuff/templates/cbuff.h.xdt",
            moduleName: cbuff_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_CBUFF",
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
};

exports = cbuff_module;
