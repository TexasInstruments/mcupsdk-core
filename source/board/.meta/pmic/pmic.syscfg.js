
let common = system.getScript("/common");
let soc = system.getScript(`/board/pmic/pmic_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.name);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getConfigurables() {
    /* get 'CPU enable' configurables */
    let config = [];
    let staticConfig = soc.getConfigArr();

    config.push(
        {
            ...common.ui.makeConfig(staticConfig, "name", "PMIC")
        }
    );

    return config;
}

let pmic_module_name = "/board/pmic/pmic";

let pmic_module = {
    displayName: "PMIC",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/pmic/templates/pmic_open_close_config.c.xdt",
            board_close: "/board/pmic/templates/pmic_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/pmic/templates/pmic_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/pmic/templates/pmic.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open: "/board/pmic/templates/pmic_open.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/board/pmic/templates/pmic_driver_open_close.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_PMIC",
    config: getConfigurables(),
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    maxInstances: 1,
    sharedModuleInstances: moduleInstances,
    getInstanceConfig,
};

function moduleInstances(instance) {
    let modInstances = new Array();
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === instance.name);

    if(config.type == "MCSPI") {
        modInstances.push({
            name: "peripheralDriver",
            displayName: "MCSPI Configuration",
            moduleName: '/drivers/mcspi/mcspi',
        });
    }

    return (modInstances);
}

exports = pmic_module;
