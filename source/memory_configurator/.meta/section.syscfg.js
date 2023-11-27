let common   = system.getScript("/common");
let book_keeping = []
const memoryRegs = system.getScript("/memory_configurator/helper");

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
            inst.split_across_memories = false;
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
            inst.select_multiple_regions = false;
        }
    },
    {
        name: "load_memory",
        displayName: "Load Memory",
        default:"",
        description:'Choose a memory region from the ones added in the Memory regions section.',
        options: () => {return memoryRegs.loadMemoryRegions(book_keeping).memory_regions},
        onChange: (inst) => { inst.run_memory = inst.load_memory
                }
    },
    {
        name: "run_memory",
        displayName: "Run Memory",
        default: "",
        description:'Choose a memory region from the ones added in the Memory regions section.',
        options: () => {return memoryRegs.loadMemoryRegions(book_keeping).memory_regions},
    },
    {
        name: "generic_text",
        displayName: "Generic Text",
        hidden: true,
        default: "",
        multiline: true,
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
    if(inst.$ownedBy === undefined && inst.output_section.length == 0) {
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

    if(inst.$ownedBy === undefined){
        modInstances.push({
            name: "output_section",
            displayName: "Output Sections",
            moduleName: "memory_configurator/output_section",
            useArray: true,
            minInstanceCount: 0,
            collapsed: false,
        });
    }
    return modInstances;
}