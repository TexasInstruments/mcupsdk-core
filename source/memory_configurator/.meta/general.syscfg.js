let common = system.getScript("/common");

function defaultValues(){
    let def_obj={
        "entry_point":"",
        "irq_stack_size":0,
        "fiq_stack_size":0,
        "svc_stack_size":0,
        "abort_stack_size":0,
        "undefined_stack_size":0
    }
    let selfCoreName = common.getSelfSysCfgCoreName();

    if(selfCoreName.includes("r5f")){
        def_obj.entry_point="-e_vectors"
        def_obj.irq_stack_size=256,
        def_obj.fiq_stack_size=256,
        def_obj.svc_stack_size=4096,
        def_obj.abort_stack_size=256,
        def_obj.undefined_stack_size=256
    }
    else  if(selfCoreName.includes("c66")){
        def_obj.entry_point="--retain=_vectors"
    }

    return def_obj;
}

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
    {
        name: "entry_point",
        displayName: "Entry Point",
        default: defaultValues().entry_point,
        description:'',
    },
]

let advanced_configs = [
    {
        name: "irq_stack_size",
        displayName: "IRQ Stack Size",
        default:defaultValues().irq_stack_size,
        description:'',
    },
    {
        name: "fiq_stack_size",
        displayName: "FIQ Stack Size",
        default:defaultValues().fiq_stack_size,
        description:'',
    },
    {
        name: "svc_stack_size",
        displayName: "SVC Stack Size",
        default:defaultValues().svc_stack_size,
        description:'',
    },
    {
        name: "abort_stack_size",
        displayName: "Abort Stack Size",
        default:defaultValues().abort_stack_size,
        description:'',
    },
    {
        name: "undefined_stack_size",
        displayName: "Undefined Stack Size",
        default:defaultValues().undefined_stack_size,
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
        longDescription: "Applicable only for R5F cores. Any change in values for other cores won't have any effect.",
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