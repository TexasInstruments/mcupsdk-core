let common   = system.getScript("/common");
let selfCoreName = common.getSelfSysCfgCoreName();
let region_module = system.modules['/memory_configurator/region'];

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
        collapsed: false,
    });

    return modInstances;
}

function validate(inst, report){

    let coreNames =  common.getSysCfgCoreNames();
    let own_mpu = inst.mpu_setting;

    for (let core of coreNames) {

        if( core.includes(selfCoreName) ) {
            continue;
        }

        let core_module = common.getModuleForCore('/memory_configurator/region', core);
        if(core_module !== undefined) {
            let others_mpu = core_module.$instances[0].mpu_setting;
            if( others_mpu !== own_mpu){
                report.logError(`MPU setting differs from ${core}. All cores should have either Automate MPU ON or OFF`, inst, "mpu_setting");
            }
        }
    }
}

exports = {
    defaultInstanceName: "MEMORY_REGION_CONFIGURATION",
	displayName: "Memory Region",
	config: config,
    maxInstances: 1,
    longDescription: '',
    moduleInstances: addModuleInstances,
    validate: validate
}