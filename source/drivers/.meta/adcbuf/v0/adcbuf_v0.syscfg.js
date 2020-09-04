
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/adcbuf/soc/adcbuf_${common.getSocName()}`).getConfigArr();
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

let adcbuf_module_name = "/drivers/adcbuf/adcbuf";

let adcbuf_module = {
    displayName: "ADCBUF",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/adcbuf/templates/adcbuf_config.c.xdt",
            driver_init: "/drivers/adcbuf/templates/adcbuf_init.c.xdt",
            driver_deinit: "/drivers/adcbuf/templates/adcbuf_deinit.c.xdt",
            moduleName: adcbuf_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/adcbuf/templates/adcbuf.h.xdt",
            moduleName: adcbuf_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_ADCBUF",
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

exports = adcbuf_module;
