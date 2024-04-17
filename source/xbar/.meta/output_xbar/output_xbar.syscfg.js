
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/output_xbar/soc/output_xbar_${common.getSocName()}`);
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getSelectedInstance(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let instance = solution.peripheralName.substr(10);

    return instance;
};

let OUTPUT_XBAR_LIST = soc.getOptionList("INTERNAL");
let OUTPUT_XBAR = JSON.parse(JSON.stringify(OUTPUT_XBAR_LIST));
OUTPUT_XBAR.map(function(item) {
    delete item.path;
    delete item.group;
    return item;
});

function getInterfaceName(inst)
{
    return soc.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    if(common.getSocName() == "am263px"){
        return [ "XBAROUT" ];
    }
    else{
        return [ "OUTPUTXBAR" ];
    }
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinName = getPeripheralPinNames(inst)[0];
    resources.push( pinmux.getPinRequirements(interfaceName, pinName, "Outputxbar Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: "OUTPUTXBAR Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}


let outputxbar_module_name = "/xbar/output_xbar/output_xbar";

let output_xbar_module = {
    displayName: "OUTPUT XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/output_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/output_xbar/templates/output_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/output_xbar/templates/output_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/output_xbar/templates/output_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/output_xbar/templates/output_xbar_open.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: outputxbar_module_name,
        },
    },
    defaultInstanceName: "CONFIG_OUTPUT_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: [],
            minSelections: 0,
            options: OUTPUT_XBAR,
            description: "This determines the output of the xbar",
        },
        {
            name: "invertLatchOutput",
            displayName: "Invert Output Before Latch",
            description: "Inverts the Xbar output before it is latched",
            default: false,
        },
        {
            name: "selectLatchSignalSource",
            displayName: "Select Latch Singal Source",
            description: "Selects the latch signal source",
            default: soc.XBAR_Latch_Source[0].name,
            options: soc.XBAR_Latch_Source,
        },
        {
            name: "selectStretchedPulseSource",
            displayName: "Select Stretched Pulse Source",
            description: "Selects the stretched pulse source",
            default: soc.XBAR_Stretched_Pulse_Source[0].name,
            options: soc.XBAR_Stretched_Pulse_Source,
        },
        {
            name: "selectStretchedPulseLength",
            displayName: "Select Stretched Pulse Length",
            description: "Selects the stretched pulse length",
            default: soc.XBAR_Stretched_Pulse_Length[0].name,
            options: soc.XBAR_Stretched_Pulse_Length,
        },
        {
            name: "invertOutput",
            displayName: "Invert XBAR Output",
            description: "Inverts the Xbar output",
            default: false,
        },
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
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
    getSelectedInstance,
    onMigrate,
};

function onMigrate(newInst, oldInst, oldSystem) {
    let pins = getPeripheralPinNames(oldInst)
    let interfaceName = getInterfaceName(oldInst)
    common.onMigrate(newInst, oldInst, oldSystem, pins, interfaceName)
}

function moduleInstances(instance) {
    let modInstances = new Array();
    var outputs = new Array();
    var flag = 0;
    for(var selections in instance.xbarOutput)
    {
        var xbarConfig = (soc.supportXbarConfig(OUTPUT_XBAR_LIST.find(o => o.name == instance.xbarOutput[selections]), instance))[0];
        if(xbarConfig != null)
        {
            flag = 1;
            outputs.push(xbarConfig.requiredArgs.xbarOutput);
        }
    }
    if(outputs != null && flag == 1)
    {
        modInstances.push({
            name: "xbarConfig",
            displayName: "EPWM_SYNCOUT_XBAR",
            moduleName: `/xbar/epwm_syncout_xbar/epwm_syncout_xbar`,
            requiredArgs: {
                xbarOutput: outputs,
            }
        });
    }
    return modInstances;
}

function validate(instance, report) {

    // If no instance is selected
    if(instance["xbarOutput"].length == 0)
    {
        report.logError("Please select atleast one input for this Xbar", instance, "xbarOutput");
    }
}

exports = output_xbar_module;
