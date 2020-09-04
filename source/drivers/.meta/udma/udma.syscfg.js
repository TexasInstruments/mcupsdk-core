
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/udma/soc/udma_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getMaxBlkCopyChannels(instance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === instance.instance);

    return config.numBlkCopyCh;
}

let udma_module = {
    displayName: "UDMA",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/udma/templates/udma_config.c.xdt",
            driver_init: "/drivers/udma/templates/udma_init.c.xdt",
            driver_deinit: "/drivers/udma/templates/udma_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/udma/templates/udma.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/udma/templates/udma_open_close_config.c.xdt",
            driver_open: "/drivers/udma/templates/udma_open.c.xdt",
            driver_close: "/drivers/udma/templates/udma_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/udma/templates/udma_open_close.h.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_UDMA",
    config: [
        common.ui.makeInstanceConfig(getConfigArr()),
        {
                name: "skipGlobalEventReg",
                displayName: "Skip Global Event Registration",
                default: false,
                description: "By default the driver allocates a single global event handle so that multiple drivers can share the same interrupt aggregator. Set this flag to TRUE to skip this registration",
        },
        {
                name: "virtToPhyFxn",
                displayName: "Virtual to Physical Callback",
                default: "Udma_defaultVirtToPhyFxn",
                description: "If non default function is used, user should define this in the application to avoid linker error",
        },
        {
                name: "phyToVirtFxn",
                displayName: "Physical to Virtual Callback",
                default: "Udma_defaultPhyToVirtFxn",
                description: "If non default function is used, user should define this in the application to avoid linker error",
        },
    ],
    validate: validate,
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
    getMaxBlkCopyChannels,
};

/*
 *  ======== validate ========
 */
function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
    common.validate.checkValidCName(instance, report, "virtToPhyFxn");
    common.validate.checkValidCName(instance, report, "phyToVirtFxn");
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(instance) {
    let modInstances = new Array();

    let maxBlkCopyCh = getMaxBlkCopyChannels(instance);
    if(maxBlkCopyCh > 0) {
        modInstances.push({
            name: "udmaBlkCopyChannel",
            displayName: "UDMA Block Copy Channel Configuration",
            moduleName: '/drivers/udma/udma_blkcopy_channel',
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
        });
    }

    return (modInstances);
}

exports = udma_module;
