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
                    else if(device == "AM263Px"){
                        if(selfCoreName.includes("r5fss")){
                            memory_regions_count = 11;
                            sections_count = 12;
                        }
                    }

                    scripting.addModule("/memory_configurator/general.syscfg.js", {}, false).addInstance()


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

                }
    }],
    maxInstances: 1,
    minInstances: 1,
}