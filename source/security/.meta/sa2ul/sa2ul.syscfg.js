
let common = system.getScript("/common");
let soc = system.getScript(`/security/sa2ul/sa2ul_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

let sa2ul_module_name = "/security/sa2ul/sa2ul";

let sa2ul_module = {
    displayName: "SA2UL",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/security/sa2ul/templates/sa2ul_config.c.xdt",
            driver_init: "/security/sa2ul/templates/sa2ul_init.c.xdt",
            driver_deinit: "/security/sa2ul/templates/sa2ul_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/security/sa2ul/templates/sa2ul.h.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_SA2UL",
    config: [
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    validate: validate,
    sharedModuleInstances: addModuleInstances,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
};

/*
 *  ======== moduleInstances ========
 */
function addModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "udmaPKTDMA",
        displayName: "UDMA PKTDMA Instance Configuration",
        description: "PKTDMA Instance to use for SA2UL",
        moduleName: '/drivers/udma/udma',
        collapsed: false,
        requiredArgs: {
            instance: "PKTDMA_0",
        },
    });

    return (modInstances);
}

/*
 *  ======== validate ========
 */
function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

exports = sa2ul_module;
