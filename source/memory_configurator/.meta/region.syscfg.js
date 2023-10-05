let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name: "mpu_setting",
        displayName: "Automate MPU Setting",
        default: false,
        longDescription: 'Checking this would save the user from manually configure MPU for regions.\
        Uncheck this to get flexibility in configuring MPU.'
    },
    // {
    //     name: "scriptingButton",
    //     displayName: "",
    //     // default: "0",
    //     buttonText: "Add Basic",
    //     scriptingOnComplete: (inst) => {
    //         //const basicInst = scripting.addModule("/memory_configurator/general.syscfg.js", {}, false).addInstance()
    //         //basicInst.cfgText = inst.from
    //         let basicInst = scripting.addModule("/memory_configurator/memory_region.syscfg.js", {}, false).addInstance()
    //         basicInst.memory_region[0].auto=false;
    //     }
    // }
]

function memoryReg(ind){
    let obj = {
        "$name":"MEMORY_REGION_CONFIGURATION2",

    }
    return obj;
}

function addModuleInstances(inst) {
    let modInstances = new Array();
    let module_name = ""

    if (inst.mpu_setting) {
        module_name = "memory_configurator/memory_region_mpu"
    }
    else {
        module_name = "memory_configurator/memory_region"
    }

    modInstances.push({
        name: "memory_region",
        displayName: "Region",
        moduleName: module_name,
        useArray: true,
        minInstanceCount: 0,
        //defaultInstanceCount:10,
        collapsed: false,
    });

    return modInstances;
}


exports = {
    defaultInstanceName: "MEMORY_REGION_CONFIGURATION",
	displayName: "Memory Region",
	config: config,
    maxInstances: 1,
    moduleInstances: addModuleInstances,
}