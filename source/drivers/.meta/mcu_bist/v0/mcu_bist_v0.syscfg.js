
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/mcu_bist/soc/mcu_bist_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

function getConfigurables() {

    let configurables = [
        {
            name: "enableMcuPbist",
            displayName: "Start MCU PBIST On System Init",
            description: `Start MCU PBIST On System_Init()`,
            default: true,
        },
    ];

    return configurables;
}

function validate(instance, report) {

    if (instance.enableMcuPbist == false)
    {
        report.logError("Enable PBIST", instance, "enableMcuPbist");
    }
}

let bist_module = {
    displayName: "MCU_BIST",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mcu_bist/templates/mcu_bist_config.c.xdt",
            driver_init: "/drivers/mcu_bist/templates/mcu_bist_init.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mcu_bist/templates/mcu_bist.h.xdt",
        },
    },
    maxInstances: 1,
    defaultInstanceName: "CONFIG_MCU_BIST",
    config: getConfigurables(),
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
};


exports = bist_module;
