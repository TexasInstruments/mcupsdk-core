
let gpio_module_name = "/drivers/gtc/gtc";

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let gtc_module = {
    displayName: "GTC",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/gtc/templates/gtc_init.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/gtc/templates/gtc.h.xdt",
        },
    },

    defaultInstanceName: "GTC",
    maxInstances: 1,

    getInstanceConfig,

};

exports = gtc_module;