let common = system.getScript("/common");

function util_function() {
        let region_module = system.modules['/memory_configurator/region'];
        let coreNames =  common.getSysCfgCoreNames();
        let regions_shared_with_this_core = []
        let memory_region_module_name = undefined

        if(region_module !== undefined) {
           if(region_module.$instances[0].mpu_setting)
               memory_region_module_name = "/memory_configurator/memory_region_mpu";
           else
               memory_region_module_name = "/memory_configurator/memory_region";

        let all_core_memory_instances = []

        let selfCoreName = common.getSelfSysCfgCoreName();

        for (let core of coreNames) {

            if( core.includes(selfCoreName) ) {
                continue;
            }

            let core_module = common.getModuleForCore(memory_region_module_name, core);
                if(core_module !== undefined) {
                    let core_module_instances = core_module.$instances;
                    all_core_memory_instances.push(core_module_instances)
                }
        }

        _.each(all_core_memory_instances, (each_core_memory_module) => {
            _.each(each_core_memory_module, (each_instance) => {
                if( each_instance.isShared && each_instance.shared_cores.includes(selfCoreName) ) {
                    regions_shared_with_this_core.push({name: each_instance.$name , displayName: each_instance.$name})
                }
            })
        })
    }

    return regions_shared_with_this_core
}

let config = [
    {
        name: "shared_region",
        displayName: "Shared Region",
        description: "",
        longDescription: "",
        default: "",
        options: util_function,
    },
    {
        name: "shared_region_name_change",
        displayName: "Name This Region",
        description: "",
        longDescription: "",
        default: "",
    },
]

function validate(inst, report){

    if(inst.shared_region_name_change.trim().length == 0 ){
        report.logError('Enter a non-empty name', inst, "shared_region_name_change")
    }
}

let shared_module = {
    displayName: "Rename Shared Regions",
    defaultInstanceName: "CONFIG_SHARED",
    config : config,
    validate: validate
}

exports = shared_module;