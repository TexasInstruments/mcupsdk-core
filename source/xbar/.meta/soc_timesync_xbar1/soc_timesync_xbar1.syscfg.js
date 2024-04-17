
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/soc_timesync_xbar1/soc/soc_timesync_xbar1_${common.getSocName()}`);

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

let SOC_TIMESYNC_XBAR1_LIST = soc.getOptionList("INTERNAL");
let SOC_TIMESYNC_XBAR1 = JSON.parse(JSON.stringify(SOC_TIMESYNC_XBAR1_LIST));
SOC_TIMESYNC_XBAR1.map(function(item) {
    delete item.path;
    return item;
});

let soc_timesync_xbar1_module = {
    displayName: "SOC TIMESYNC XBAR 1",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/soc_timesync_xbar1.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/soc_timesync_xbar1/templates/soc_timesync_xbar1.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/soc_timesync_xbar1/templates/soc_timesync_xbar1_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/soc_timesync_xbar1/templates/soc_timesync_xbar1_open_close_config.c.xdt",
            driver_open: "/xbar/soc_timesync_xbar1/templates/soc_timesync_xbar1_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_SOC_TIMESYNC_XBAR1_",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: SOC_TIMESYNC_XBAR1[0].name,
            options: SOC_TIMESYNC_XBAR1,
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
    let instanceName = instance.instance;
    if(instance.$ownedBy)
    {
        let masterPath = instance.$ownedBy.$module.$name;
        let masterXbar = masterPath.substring(masterPath.lastIndexOf("/") + 1).toUpperCase();
        if(instanceName.search(masterXbar) == -1)
        {
            report.logError(`Use correct xbar instance`, instance, "instance");
        }
    }
    common.validate.checkSameInstanceNameOnAllCores('/xbar/soc_timesync_xbar1/soc_timesync_xbar1', instance, report);
}

function moduleInstances(instance) {
    return soc.supportXbarConfig(SOC_TIMESYNC_XBAR1_LIST.find(o => o.name == instance.xbarOutput), instance);
}

exports = soc_timesync_xbar1_module;
