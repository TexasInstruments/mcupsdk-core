
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/gpio_int_xbar/soc/gpio_int_xbar_${common.getSocName()}`);

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

let GPIO_INT_XBAR_LIST = soc.getOptionList("INTERNAL");
let GPIO_INT_XBAR = JSON.parse(JSON.stringify(GPIO_INT_XBAR_LIST));
GPIO_INT_XBAR.map(function(item) {
    delete item.path;
    return item;
});

let gpio_int_xbar_module = {
    displayName: "GPIO INT XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/gpio_int_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/gpio_int_xbar/templates/gpio_int_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/gpio_int_xbar/templates/gpio_int_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/gpio_int_xbar/templates/gpio_int_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/gpio_int_xbar/templates/gpio_int_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_GPIO_INT_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: GPIO_INT_XBAR[0].name,
            options: GPIO_INT_XBAR,
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
    common.validate.checkSameInstanceNameOnAllCores('/xbar/gpio_int_xbar/gpio_int_xbar', instance, report);
}

exports = gpio_int_xbar_module;
