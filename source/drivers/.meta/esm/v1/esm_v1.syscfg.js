
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/esm/soc/esm_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

     return {
        ...config,
        ...moduleInstance,
     };
};

function getMaxNotifier(inst) {
    return soc.getMaxNotifier(inst);
}

let esm_module_name = "/drivers/esm/esm";
let esm_module = {
    displayName: "ESM",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/esm/templates/esm_config_v1.c.xdt",
            driver_init: "/drivers/esm/templates/esm_init.c.xdt",
            driver_deinit: "/drivers/esm/templates/esm_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/esm/templates/esm.h.xdt",
        },

        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/esm/templates/esm_open_close_config_v1.c.xdt",
            driver_open: "/drivers/esm/templates/esm_open.c.xdt",
            driver_close: "/drivers/esm/templates/esm_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/esm/templates/esm_open_close.h.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_ESM",
    config: [
        {
            name: "bClearErrors",
            displayName: "Clear Group Errors",
            default: false,
            description: "Clear group 1, 2 and 3 errors",
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    moduleInstances: moduleInstances,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    let maxNotifier = getMaxNotifier(inst);
    modInstances.push({
        name: "esmNotifier",
        displayName: "ESM Notifier Configuration",
        moduleName: '/drivers/esm/v1/esm_v1_notifier',
        useArray: true,
        maxInstanceCount: maxNotifier,
        minInstanceCount: 1,
        defaultInstanceCount: 1,
    });

    return (modInstances);
}

exports = esm_module;
