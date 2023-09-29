let common   = system.getScript("/common");
let book_keeping = []
function loadMemoryRegions () {

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
                       opti_share: false
                       }
                       let displayName = instance.$name.concat("; Type: ",instance.type)
                       //displayName = displayName.concat("Opti-share ", instance.opti_share)
                       if (core.includes(selfCoreName)) {
                           memory_regions.push({name: instance.$name, displayName: displayName})
                           obj.name = instance.$name
                           obj.opti_share = instance.opti_share
                       }
                       else if(instance.isShared && instance.shared_cores.includes(selfCoreName)) {
                           memory_regions.push({name: instance.$name, displayName: displayName})
                           obj.name = instance.$name
                           obj.opti_share = instance.opti_share
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

   let config = [
    {
        name: "$name",
        isCIdentifier: false
    },
    {
        name: "load_memory",
        displayName: "Load Memory",
        default:"",
        description:'Choose a memory region from the ones added in the Memory regions section.',
        options: () => {return loadMemoryRegions().memory_regions},
        //onChange: (inst) => {inst.run_memory = inst.load_memory}
    },
    // {
    //     name: "run_memory",
    //     displayName: "Run Memory",
    //     default:"",
    //     description:'Choose a memory region from the ones added in the Memory regions section.',
    //     options: () => {return loadMemoryRegions().memory_regions},
    // },
]

function validate(inst, report) {

    if(inst.load_memory.length == 0) {
        report.logError("This field can't be kept empty", inst, "load_memory")
    }
    // if(inst.run_memory.length == 0) {
    //     report.logError("This field can't be kept empty", inst, "run_memory")
    // }
}

exports= {
    defaultInstanceName: "Memory Region",
	displayName: "Memory Region",
	config: config,
    validate: validate
}