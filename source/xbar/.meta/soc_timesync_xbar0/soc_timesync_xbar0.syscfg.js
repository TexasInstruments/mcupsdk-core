
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/soc_timesync_xbar0/soc/soc_timesync_xbar0_${common.getSocName()}`);

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

let SOC_TIMESYNC_XBAR0_LIST = soc.getOptionList("INTERNAL");
let SOC_TIMESYNC_XBAR0 = JSON.parse(JSON.stringify(SOC_TIMESYNC_XBAR0_LIST));
SOC_TIMESYNC_XBAR0.map(function(item) {
    delete item.path;
    return item;
});

let soc_timesync_xbar0_module = {
    displayName: "SOC TIMESYNC XBAR 0",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/soc_timesync_xbar0.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/soc_timesync_xbar0/templates/soc_timesync_xbar0.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/soc_timesync_xbar0/templates/soc_timesync_xbar0_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/soc_timesync_xbar0/templates/soc_timesync_xbar0_open_close_config.c.xdt",
            driver_open: "/xbar/soc_timesync_xbar0/templates/soc_timesync_xbar0_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_SOC_TIMESYNC_XBAR0_",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: SOC_TIMESYNC_XBAR0[0].name,
            options: SOC_TIMESYNC_XBAR0,
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
    common.validate.checkSameInstanceNameOnAllCores('/xbar/soc_timesync_xbar0/soc_timesync_xbar0', instance, report);
}

exports = soc_timesync_xbar0_module;
