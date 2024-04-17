
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/icl_xbar/soc/icl_xbar_${common.getSocName()}`);

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

let ICL_XBAR_LIST = soc.getOptionList("INTERNAL");
let ICL_XBAR = JSON.parse(JSON.stringify(ICL_XBAR_LIST));
ICL_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

let icl_xbar_module = {
    displayName: "ICL XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/icl_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/icl_xbar/templates/icl_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/icl_xbar/templates/icl_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/icl_xbar/templates/icl_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/icl_xbar/templates/icl_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_ICL_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: [],
            minSelections: 0,
            options: ICL_XBAR,
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
    common.validate.checkSameInstanceNameOnAllCores('/xbar/icl_xbar/icl_xbar', instance, report);

    // If no instance is selected
    if(instance["xbarOutput"].length == 0)
    {
        report.logError("Please select atleast one input for this Xbar", instance, "xbarOutput");
    }
}

exports = icl_xbar_module;
