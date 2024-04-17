
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/icss_xbar/soc/icss_xbar_${common.getSocName()}`);

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

let ICSS_XBAR_LIST = soc.getOptionList("INTERNAL");
let ICSS_XBAR = JSON.parse(JSON.stringify(ICSS_XBAR_LIST));
ICSS_XBAR.map(function(item) {
    delete item.path;
    return item;
});

let icss_xbar_module = {
    displayName: "ICSS XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/icss_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/icss_xbar/templates/icss_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/icss_xbar/templates/icss_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/icss_xbar/templates/icss_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/icss_xbar/templates/icss_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_ICSS_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: ICSS_XBAR[0].name,
            options: ICSS_XBAR,
            description: "This determines the output of the xbar",
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    validate: validate,
    getInstanceConfig,
};

function validate(instance, report) {
    common.validate.checkSameInstanceNameOnAllCores('/xbar/icss_xbar/icss_xbar', instance, report);
}

function moduleInstances(instance) {
    return soc.supportXbarConfig(ICSS_XBAR_LIST.find(o => o.name == instance.xbarOutput), instance);
}

exports = icss_xbar_module;
