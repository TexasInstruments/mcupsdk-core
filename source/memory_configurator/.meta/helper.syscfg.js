 const common = system.getScript("/common");
 const selfCoreName = common.getSelfSysCfgCoreName();
 const module = system.modules['/kernel/dpl/mpu_armv7'];
 const physicalLayout = system.getScript("/memory_configurator/physicalLayout.json")[system.deviceData.device];

 function checkNameChange( name ){

    let selfCoreName = common.getSelfSysCfgCoreName();
    let selfCore_shared_module = common.getModuleForCore("/memory_configurator/shared_region_references", selfCoreName);
    let newName = name;

    if( selfCore_shared_module !== undefined){
        let selfCore_shared_instances = selfCore_shared_module.$instances
        _.each(selfCore_shared_instances, (each_instance) => {
            if ( each_instance.shared_region === name){
                newName =  each_instance.shared_region_name_change
            }
        })
    }
    return newName

}


function memoryRegionInformation(mpu_instance, mpu_set) {

        let mr_list = {
            name: [],
            type: [],
            start_addr: [],
            size: [],
            end_addr: [],
            permissions: [],
            shared: [],
            cores: [],
            access_check: []
        }
        let all_core_mr_instances = util_function(mpu_set)
        let mr_instances = []
        _.each(all_core_mr_instances, (each_core_memory_module) => {
                if(each_core_memory_module.core.length > 0 ) {
                    _.each(each_core_memory_module.instances, (inst) => {
                        let obj = Object.assign({}, inst) ;
                        obj.core = each_core_memory_module.core
                        mr_instances.push(obj)
                    })
                }
        })
        let sorted_memory_regions = _.chain(mr_instances)
        .sortBy((mr_instance) => (mr_instance.auto ? mr_instance.autoStartAddress: mr_instance.manualStartAddress))
        .value();

        let mp_start = mpu_instance.baseAddr
        let mp_size = Math.pow(2, mpu_instance.size)
        let mp_end = Number(mp_start) + Number(mp_size)
        _.each(sorted_memory_regions, (region) => {
            let mr_start = ( region.auto ? region.autoStartAddress: region.manualStartAddress )
            let mr_end =  mr_start + region.size - 0x1
            if (mr_end < mp_start || mp_end < mr_start)  // current mp instance doesn't fall inside this mr
            { }
            else
            {
                let region_name = (region.$name).concat("_",region.core)
                mr_list["name"].push(displayName(region_name))
                mr_list["type"].push(region.type)
                mr_list["start_addr"].push(mr_start)
                mr_list["size"].push(region.size)
                mr_list["end_addr"].push(mr_end)
                mr_list["permissions"].push(JSON.stringify(region.attributes))
                mr_list["cores"].push(region.core)

                if(region.isShared)
                {
                    let arr_cores = region.shared_cores

                    if(!region.core.includes(selfCoreName) && !arr_cores.includes(selfCoreName)) {
                        mr_list.access_check = false;
                    }

                    _.each( arr_cores, (core) => {
                         let last_value = mr_list["cores"][mr_list.cores.length-1]
                         let updated_value = ""
                        if(last_value.includes(core) == false){
                           updated_value = updated_value.concat(core, " ")
                            last_value = updated_value
                        }
                        if(!updated_value.includes(selfCoreName))
                            updated_value = selfCoreName.concat(" ", updated_value)
                        mr_list["cores"][mr_list.cores.length-1] = updated_value
                    } )
                   mr_list["shared"].push(true);
                }
               else{
                   mr_list["shared"].push(false);
               }
            }
        })
        return mr_list;
}


function util_function(mpu_set) {

    let coreNames =  common.getSysCfgCoreNames();
    let memory_region_module_name = '/memory_configurator/memory_region'+mpu_set

    let all_core_memory_module = []

    for (let core of coreNames) {

        let obj = {
            instances: [],
            core: "",
        }
        let core_module = common.getModuleForCore(memory_region_module_name, core);
            if(core_module != undefined) {
                let core_module_instances = core_module.$instances;
                obj.instances = core_module_instances
                obj.core = core
            }
            all_core_memory_module.push(obj)
    }

    return all_core_memory_module
}

function displayName(name) {

    let lastIndex = name.lastIndexOf("_");
    let region_name = name.substring(0, lastIndex);
    let core_name = name.substring(lastIndex+1);

    return region_name;
}

function loadMemoryRegions (book_keeping) {

    /*  List should include all the MRs defined in this core as well as shared by other cores with this core*/
       let memory_regions = []

       let coreNames =  common.getSysCfgCoreNames();
       let memory_region_module_name = ""
       let region_module = system.modules['/memory_configurator/region'];
       if(region_module !== undefined) {
           if(region_module.$instances[0].mpu_setting)
               memory_region_module_name = "/memory_configurator/memory_region_mpu";
           else
               memory_region_module_name = "/memory_configurator/memory_region";

           let selfCoreName = common.getSelfSysCfgCoreName();
           book_keeping.splice(0, book_keeping.length)
           for (let core of coreNames) {

               let core_module = common.getModuleForCore(memory_region_module_name, core);
               let core_module_instances;
               if(core_module != undefined) {
                   core_module_instances = core_module.$instances;
                   _.each(core_module_instances, instance => {

                       let obj = { name: " ",
                       }
                       let displayName = instance.$name.concat(" Type: ",instance.type)
                       if (core.includes(selfCoreName)) {
                           memory_regions.push({name: instance.$name, displayName: displayName})
                           obj.name = instance.$name
                       }
                       else if(instance.isShared && instance.shared_cores.includes(selfCoreName)) {

                           let newName = checkNameChange(instance.$name)
                           let displayName_changed = newName.concat(" Type: ",instance.type)
                           memory_regions.push({name: instance.$name, displayName: displayName_changed})
                           obj.name = instance.$name
                       }
                       book_keeping.push(obj)
                   })
               }
           }
       }
       else {
           memory_regions.push({name: "", displayName: ""})
       }
    return {memory_regions}
}

function coreList(inst) {
    let coreNames = common.getSysCfgCoreNames();
    let selfCoreName = common.getSelfSysCfgCoreName();

    let tmp_list = coreNames.filter(function (coreName) {
        return coreName !== selfCoreName;
    });

    let memoryRegType = inst.type;
    let applicable_cores = (memoryRegType.slice(memoryRegType.lastIndexOf('_')+1)).toLowerCase();
    if ( memoryRegType.indexOf('_')== -1 ) memoryRegType = memoryRegType+"_ALL"; // This is for types which do not follow the _ALL type format, it's assumed that this region could be shared among all

    let options = [];

    for (let core of tmp_list)
    {
        let ele = {name: core, displayName: core}
        if( memoryRegType.includes("_ALL") ) // This means that the memory type can be accessed across all cores
            options.push(ele)
        else if ( core.includes(applicable_cores) && physicalLayout[memoryRegType].access === "all" ) //Display only those which cores with which the current core can access given the memory time
            options.push(ele)
    }
	return options;
}

function computeCoreNum()
{
    let options = coreList();
    let default_shared_cores = [];

    for(let i = 0; i <  options.length; i++){
        default_shared_cores.push(options[i].name);
    }

    return default_shared_cores;
}

exports = {
    memoryRegionInformation,
    util_function,
    displayName,
    loadMemoryRegions,
    coreList
}