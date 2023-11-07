let common = system.getScript("/common");
let selfCoreName = common.getSelfSysCfgCoreName();
const device = system.deviceData.device;

exports = {
    displayName: "Default Linker Config",
	config: [
        {
            name: "scriptingButton",
            displayName: "Add Default Linker Config",
            buttonText: "CLICK",
            scriptingOnComplete: (inst) => {
                    let memory_regions_count = 0
                    let sections_count = 0

                    if(device == "AM273x"){
                        if(selfCoreName.includes("r5fss0-0") || selfCoreName.includes("r5fss0-1")){
                            memory_regions_count = 10;
                            sections_count = 12;
                        }
                        else if (selfCoreName.includes("c66")){
                            memory_regions_count = 7;
                            sections_count = 10;
                        }
                    }
                    else if(device == "AM263x_beta") {
                        if(selfCoreName.includes("r5fss")){
                            memory_regions_count = 11;
                            sections_count = 12;
                        }
                    }
                    else if(device == "AM263Px"){
                        if(selfCoreName.includes("r5fss")){
                            memory_regions_count = 11;
                            sections_count = 12;
                        }
                    }
                    else if(device == "AM64x" || device == "AM243x_ALV_beta" || device == "AM243x_ALX_beta") {
                        if(selfCoreName.includes("r5fss")){
                            memory_regions_count = 9;
                            sections_count = 11;
                        }
                        else if(selfCoreName.includes("m4f")){
                            memory_regions_count = 6;
                            sections_count = 8;
                        }
                        else if(selfCoreName.includes("a53")){
                            memory_regions_count = 4;
                            sections_count = 7;
                        }
                    }

                    let generalInst = scripting.addModule("/memory_configurator/general.syscfg.js", {}, false).addInstance()


                    let regionInst = scripting.addModule("/memory_configurator/region.syscfg.js", {}, false).addInstance()
                    regionInst.memory_region.create(memory_regions_count);
                    for(let i=0;i<memory_regions_count;i++){
                        system.getScript("/memory_configurator/default_linker_config").populate_memory_regions(regionInst, i, device, selfCoreName)
                    }


                    let sectionMod = scripting.addModule("/memory_configurator/section.syscfg.js", {}, false)
                    for(let i=0;i<sections_count;i++){
                        let sectionInst= sectionMod.addInstance()
                        system.getScript("/memory_configurator/default_linker_config").populate_sections_regions(sectionInst, i+1, device, selfCoreName)
                    }

                    if (selfCoreName.includes("m4f")){

                        let coreNames =  common.getSysCfgCoreNames();
                        let region_module = system.modules['/memory_configurator/region'];
                        let memory_region_module_name = undefined
                        let isShared = false
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
                                        isShared = true;
                                        return;
                                    }
                                })
                            })
                        }

                        if(isShared){

                            let sharedRegionNameChangeMod = scripting.addModule("/memory_configurator/shared_region_references", {}, false);
                            let sharedRegionNameChangeInst= sharedRegionNameChangeMod.addInstance()
                            system.getScript("/memory_configurator/default_linker_config").populate_shared_region_name_change(sharedRegionNameChangeInst, device, selfCoreName);
                        }
                    }

                }
    }],
    maxInstances: 1,
    minInstances: 1,
}