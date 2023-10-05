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
                    let displayName = instance.$name.concat(" Type: ",instance.type)
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

function isOptiShare(inst) {

    let memory_arr = book_keeping
    let value = false;

    _.each(memory_arr, item => {
        if( item.name === inst.load_memory || item.name === inst.run_memory) {
            // inst.$uiState.output_sections_opti.hidden = !item.opti_share;
            // inst.$uiState.input_sections_opti.hidden = !item.opti_share;
            // inst.$uiState.output_sections.hidden = item.opti_share;
            // inst.$uiState.input_sections.hidden = item.opti_share;
            // inst.opti_share = item.opti_share
            // return item.opti_share
            value = item.opti_share
    }
    })

    return value;
    // let status = false;

    // if(inst.load_memory.includes("true")) {
    //     status = true;
    // }
    //     // inst.$uiState.output_sections_opti.hidden = !status;
    //     // inst.$uiState.input_sections_opti.hidden = !status;
    //     // inst.$uiState.output_sections.hidden = status;
    //     // inst.$uiState.input_sections.hidden = status;
    // inst.opti_share = status
}

function updateOptiShareConfig(inst) {

    let status = inst.opti_share
    inst.$uiState.output_sections_opti.hidden = !status;
    inst.$uiState.input_sections_opti.hidden = !status;
    inst.$uiState.output_sections.hidden = status;
    inst.$uiState.input_sections.hidden = status;
}


let config = [
    {
        name: "$name",
        isCIdentifier: false
    },
    {
        name: "type",
        displayName: "Type",
        default:"LOAD",
        description:'',
        options: [{ name: "LOAD" }, { name: "DSECT" }, { name: "COPY" }, { name: "NOLOAD" }, { name: "NOINT" }],
    },
    {
        name: "group",
        displayName: "Group Section",
        default: true,
        longDescription:'Check this if all the output sections need to be grouped else uncheck.',
        onChange: (inst) => {
            inst.$uiState.group_start.hidden = !inst.group
            inst.$uiState.group_end.hidden = !inst.group
        }
    },
    {
        name: "group_start",
        //multiline: true,
        displayName: "Start Group",
        placeholder: "__IRQ_STACK_START",
        default:"",
        description:'This field is optional',
    },
    {
        name: "group_end",
        //multiline: true,
        displayName: "End Group",
        placeholder: "__IRQ_STACK_END",
        default:"",
        description:'This field is optional',
    },
    {
        name: "opti_share",
        displayName: "Opti Share",
        default: false,
        hidden: true,
        description:'',
        getValue: isOptiShare,
    },
    {
        name: "load_to_memory",
        displayName: "Load To Memory Region OR Address?",
        default: "Memory",
        description: "Load to any added memory region or hard code the address?",
        options: [{name: "Memory"}, {name: "Address"}],
        onChange: (inst) => {
            if(inst.load_to_memory == "Address"){
                inst.$uiState.load_memory.hidden = true;
                inst.$uiState.run_memory.hidden = true;
                inst.$uiState.split_across_memories.hidden = true;
                inst.$uiState.select_multiple_regions.hidden = true;
                inst.$uiState.load_to_address.hidden = false;
                inst.$uiState.run_at_address.hidden = false;
                //inst.$uiState.split_priority[0]["load_memory"].hidden = true;
            }
            else{
                inst.$uiState.load_memory.hidden = false;
                inst.$uiState.run_memory.hidden = false;
                inst.$uiState.split_across_memories.hidden = false;
                inst.$uiState.select_multiple_regions.hidden = false;
                inst.$uiState.load_to_address.hidden = true;
                inst.$uiState.run_at_address.hidden = true;
            }
        }
    },
    {
        name: "load_to_address",
        displayName: "Load Address",
        hidden: true,
        default: "0x0",
        description: "Write the address to where this section needs to be loaded",
        onChange: (inst) => { inst.run_at_address = inst.load_to_address}
    },
    {
        name: "run_at_address",
        displayName: "Run Address",
        hidden: true,
        default: "0x0",
        description: "Write the address to where this section needs to be ran. Can be left blank.",
    },
    {
        name: "select_multiple_regions",
        displayName: "Select Multiple Memory Regions",
        default: false,
        description:'Section will be placed in the first memory region (selected in the order below) big enough to fit it.',
        onChange: (inst) => {
            inst.$uiState.load_memory.hidden = inst.select_multiple_regions;
            inst.$uiState.run_memory.hidden = inst.select_multiple_regions;
            inst.$uiState.split_across_memories.hidden = inst.select_multiple_regions;
        }
    },
    {
        name: "split_across_memories",
        displayName: "Split Across Memories",
        default: false,
        description:'Section will split across the memory regions in the order selected below.',
        onChange: (inst) => {
            inst.$uiState.load_memory.hidden = inst.split_across_memories;
            inst.$uiState.run_memory.hidden = inst.split_across_memories;
            inst.$uiState.select_multiple_regions.hidden = inst.split_across_memories;
        }
    },
    {
        name: "load_memory",
        displayName: "Load Memory",
        default:"",
        description:'Choose a memory region from the ones added in the Memory regions section.',
        options: () => {return loadMemoryRegions().memory_regions},
        onChange: (inst) => { inst.run_memory = inst.load_memory
                }
    },
    {
        name: "run_memory",
        displayName: "Run Memory",
        default: "",
        description:'Choose a memory region from the ones added in the Memory regions section.',
        options: () => {return loadMemoryRegions().memory_regions},
    },
]

function validate(inst, report) {

    // if(inst.output_sections.length == 0) {
    //     report.logError("This field can't be kept empty", inst, "output_sections")
    // }
    if(inst.load_memory.length == 0 && !inst.split_across_memories && !inst.select_multiple_regions && inst.load_to_memory=="Memory") {
        report.logError("This field can't be kept empty", inst, "load_memory")
    }
    if(inst.run_memory.length == 0 && !inst.split_across_memories && !inst.select_multiple_regions && inst.load_to_memory=="Memory") {
        report.logError("This field can't be kept empty", inst, "run_memory")
    }
    if(inst.output_section.length == 0) {
        report.logError("Add atleast 1 output section", inst, "output_section")
    }
}
exports = {
    defaultInstanceName: "CONFIG_SECTION",
	displayName: "Section",
	config: config,
    moduleInstances: addModuleInstances,
    validate: validate
}



function addModuleInstances(inst) {
    let modInstances = new Array();

    if(inst.load_to_memory == "Memory" && (inst.split_across_memories || inst.select_multiple_regions) ){

        modInstances.push({
            name: "split_priority",
            displayName: "Priority",
            moduleName: "memory_configurator/memory_region_list",
            useArray: true,
            minInstanceCount: 1,
            collapsed: false,
        });
    }

    modInstances.push({
        name: "output_section",
        displayName: "Output Sections",
        moduleName: "memory_configurator/output_section",
        useArray: true,
        minInstanceCount: 0,
        collapsed: false,
    });



    return modInstances;
}