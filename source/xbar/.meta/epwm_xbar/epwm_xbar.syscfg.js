
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/epwm_xbar/soc/epwm_xbar_${common.getSocName()}`);

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

let EPWM_XBAR_LIST = soc.getOptionList("INTERNAL");
let EPWM_XBAR = JSON.parse(JSON.stringify(EPWM_XBAR_LIST));
EPWM_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

let epwm_xbar_module = {
    displayName: "EPWM XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/epwm_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/epwm_xbar/templates/epwm_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/epwm_xbar/templates/epwm_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/epwm_xbar/templates/epwm_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/epwm_xbar/templates/epwm_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_EPWM_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: [],
            minSelections: 0,
            options: EPWM_XBAR,
            description: "This determines the output of the xbar",
        },
        {
            name: "invertOutput",
            displayName: "Invert Output Before Latch",
            description: "Inverts the Xbar output before it is latched",
            default: false,
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

function moduleInstances(instance) {
    let modInstances = new Array();
    let i = 0;
    for(var selections in instance.xbarOutput)
    {
        var xbarConfig = (soc.supportXbarConfig(EPWM_XBAR_LIST.find(o => o.name == instance.xbarOutput[selections]), instance))[0];
        if(xbarConfig != null)
        {
            xbarConfig.name = xbarConfig.name + "-" + i;
            modInstances.push(xbarConfig);
            i++;
        }
    }
    return modInstances;
}

function validate(instance, report) {
    common.validate.checkSameInstanceNameOnAllCores('/xbar/epwm_xbar/epwm_xbar', instance, report);

    // If no instance is selected
    if(instance["xbarOutput"].length == 0)
    {
        report.logError("Please select atleast one input for this Xbar", instance, "xbarOutput");
    }
}

exports = epwm_xbar_module;
