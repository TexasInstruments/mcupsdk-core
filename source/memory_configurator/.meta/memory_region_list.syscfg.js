let common   = system.getScript("/common");
let book_keeping = []
const memoryRegs = system.getScript("/memory_configurator/helper");

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
        options: () => {return memoryRegs.loadMemoryRegions(book_keeping).memory_regions},
    },
]

function validate(inst, report) {

    if(inst.load_memory.length == 0) {
        report.logError("This field can't be kept empty", inst, "load_memory")
    }
}

exports= {
    defaultInstanceName: "Memory Region",
	displayName: "Memory Region",
	config: config,
    validate: validate
}