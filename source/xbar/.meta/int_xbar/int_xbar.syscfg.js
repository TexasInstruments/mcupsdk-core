
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/int_xbar/soc/int_xbar_${common.getSocName()}`);

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

let INT_XBAR_LIST = soc.getOptionList("INTERNAL");
let INT_XBAR = JSON.parse(JSON.stringify(INT_XBAR_LIST));
INT_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

let int_xbar_module = {
    displayName: "INT XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/int_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/int_xbar/templates/int_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/int_xbar/templates/int_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/int_xbar/templates/int_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/int_xbar/templates/int_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_INT_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: [],
            minSelections: 0,
            options: INT_XBAR,
            description: "This determines the output of the xbar",
        },
        {
            name: "parentName",
            displayName: "Owned By",
            hidden: true,
            default: "NONE",
            description: "This determines the parent module of the xbar",
            onChange: function (inst,ui){
                if(inst.parentName != "NONE")
                {
                    ui.xbarOutput.hidden = true,
                    ui.parentName.hidden = false
                }
                else
                {
                    ui.xbarOutput.hidden = false,
                    ui.parentName.hidden = true
                }
            },
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
    common.validate.checkSameInstanceNameOnAllCores('/xbar/int_xbar/int_xbar', instance, report);

    // If no instance is selected
    if(instance["xbarOutput"].length == 0 && instance.parentName == "NONE")
    {
        report.logError("Please select atleast one input for this Xbar", instance, "xbarOutput");
    }
}

exports = int_xbar_module;
