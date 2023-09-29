let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name: "choose_compiler",
        displayName: "Choose Compiler",
        default: "tiarmclang",
        options: [{name: "tiarmclang", displayName: "TIARMCLANG"}],
    },
    {
        name: "stack_size",
        displayName: "Stack Size",
        default:16384,
        description:'',
    },
    {
        name: "heap_size",
        displayName: "Heap Size",
        default:32768,
        description:'',
    },
]

let advanced_configs = [
    {
        name: "irq_stack_size",
        displayName: "IRQ Stack Size",
        default:256,
        description:'',
    },
    {
        name: "fiq_stack_size",
        displayName: "FIQ Stack Size",
        default:256,
        description:'',
    },
    {
        name: "svc_stack_size",
        displayName: "SVC Stack Size",
        default:4096,
        description:'',
    },
    {
        name: "abort_stack_size",
        displayName: "Abort Stack Size",
        default:256,
        description:'',
    },
    {
        name: "undefined_stack_size",
        displayName: "Undefined Stack Size",
        default:256,
        description:'',
    },
]

config = config.concat([
    {
        name: "additional_data",
        displayName: "Additional Data",
        description: "Add text to be written in the linker.cmd. Press enter for next line.",
        multiline: true,
        default:"",
        placeholder: " #define TASK_SIZE	0x8000 \n #define TEXT_SIZE	0x26000"
    }
])

config = config.concat([
    {
        name: "group_advanced",
        displayName: "Advanced Tab",
        description: "",
        config: advanced_configs,
    }
]);

function validate(inst, report){

    let itr = -1;
    _.each(Object.values(inst), item => {
        itr++
        let key = Object.keys(inst)[itr]
        if(key != undefined && !key.includes('$')){
            if(item < 0) {
                report.logError( `Value can't be negative`, inst, key);
            }
       }
    })
}

function addModuleInstances(inst) {
    let modInstances = new Array();
    let module_name = ""

    if (inst.choose_compiler == "tiarmclang") {
        module_name = "memory_configurator/linker_tiarmclang"
    }
    else if (inst.choose_compiler == "gcc") {
        module_name = "memory_configurator/linker_gcc"
    }

    modInstances.push({
        name: "linker",
        displayName: "Linker File",
        moduleName: module_name,
    });

    return modInstances;
}

exports = {
    defaultInstanceName: "CONFIG_GENERAL",
	displayName: "General",
	config: config,
    maxInstances: 1,
    validate: validate,
    moduleInstances: addModuleInstances,
}