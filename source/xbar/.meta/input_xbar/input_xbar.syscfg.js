
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/input_xbar/soc/input_xbar_${common.getSocName()}`);

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

let INPUT_XBAR_LIST = soc.getOptionList("INTERNAL");
let INPUT_XBAR = JSON.parse(JSON.stringify(INPUT_XBAR_LIST));
INPUT_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

let input_xbar_module = {
    displayName: "INPUT XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/input_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/input_xbar/templates/input_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/input_xbar/templates/input_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/input_xbar/templates/input_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/input_xbar/templates/input_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_INPUT_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: INPUT_XBAR[0].name,
            options: INPUT_XBAR,
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
    common.validate.checkSameInstanceNameOnAllCores('/xbar/input_xbar/input_xbar', instance, report);
}

exports = input_xbar_module;
