
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/epwm_syncout_xbar/soc/epwm_syncout_xbar_${common.getSocName()}`);

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

let EPWM_SYNCOUT_XBAR_LIST = soc.getOptionList("INTERNAL");
let EPWM_SYNCOUT_XBAR = JSON.parse(JSON.stringify(EPWM_SYNCOUT_XBAR_LIST));
EPWM_SYNCOUT_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

let epwm_syncout_xbar_module = {
    displayName: "EPWM SYNCOUT XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/epwm_syncout_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/epwm_syncout_xbar/templates/epwm_syncout_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/epwm_syncout_xbar/templates/epwm_syncout_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/epwm_syncout_xbar/templates/epwm_syncout_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/epwm_syncout_xbar/templates/epwm_syncout_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_EPWM_SYNCOUT_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: [],
            minSelections: 0,
            options: EPWM_SYNCOUT_XBAR,
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
    validate: validate,
    getInstanceConfig,
};

function validate(instance, report) {
    common.validate.checkSameInstanceNameOnAllCores('/xbar/epwm_syncout_xbar/epwm_syncout_xbar', instance, report);

    // If no instance is selected
    if(instance["xbarOutput"].length == 0)
    {
        report.logError("Please select atleast one input for this Xbar", instance, "xbarOutput");
    }
}

exports = epwm_syncout_xbar_module;
