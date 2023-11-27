let common = system.getScript("/common");
let drivers = system.getScript("/drivers/drivers")
let general_module = system.modules['/memory_configurator/general'];
const { getMemoryLayout } = system.getScript("/memory_configurator/memoryLayoutSolver");
const physicalLayout = system.getScript("/memory_configurator/physicalLayout.json")[system.deviceData.device];
const memoryRegs = system.getScript("/memory_configurator/helper");

let g_memory_region_types = [];

function memoryRegions(){

    let memory_region_types = [];
    let selfCoreName = common.getSelfSysCfgCoreName();

    for(let key in physicalLayout){
        // value = physicalLayout[key];
        core = key.slice(key.lastIndexOf('_')+1);

        if( core == "ALL" || selfCoreName.includes(core.toLowerCase()) || (key.indexOf('_')==-1) ) {
            let displayName=""
            if (key.indexOf('_')==-1){
                displayName = key;
            }
            else {
                displayName = key.slice(0,key.lastIndexOf('_'));
            }
            memory_region_types.push({name: key.toString(), displayName: displayName})
        }
    }

    g_memory_region_types = memory_region_types
    return memory_region_types;
}


function enableShared_core(inst, ui) {

    if(inst.isShared)
        ui.shared_cores.hidden = false;
    else{
        ui.shared_cores.hidden = true;
        inst.shared_cores = []
    }
}


let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false,
    },
    {
    name: "type",
    displayName: "Type",
    default: memoryRegions()[0].name,
    options: memoryRegions(),
    onChange: (inst) => {
        inst.$name = (inst.type.concat("_x")).toUpperCase()
        inst.manualStartAddress = physicalLayout[inst.type].start
        inst.isShared = false;
        inst.$uiState.shared_cores.hidden = true;
        inst.shared_cores = [];
        if(physicalLayout[inst.type].access == "individual"){
            inst.$uiState.isShared.hidden = true;
            //inst.shared_cores = [];
        }
        else {
            inst.$uiState.isShared.hidden = false;
        }
    },
    longDescription: 'Choose CUSTOM if the memory address lies outside of the remaining types.'
}, {
    name: "attributes",
    displayName: "Attributes",
    default: ["R", "W", "X", "I"],
    options: [{name:"R", displayName:"Read"}, {name:"W", displayName:"Write"}, {name:"X", displayName:"Execute"}, {name:"I", displayName:"Initialize"}],
    longDescription:"R: memory can be read; W: memory can be written to; X: memory can contain executable code, I: memory can be initialized."
},{
    name: "auto",
    displayName: "Calculate Start Address",
    default: true,
    onChange: (inst) => {
        inst.$uiState.autoStartAddress.hidden = !inst.auto;
        inst.$uiState.manualStartAddress.hidden = inst.auto;
    },
    longDescription:'Check this if placement of the region does not matter. The region will be placed in the smallest hole that fits it.'
}, {
    name: "autoStartAddress",
    displayName: "Start Address",
    default: 0x0,
    displayFormat: "hex",
    getValue: (inst) => {
        return getMemoryLayout()?.[system.context]?.[inst.$name] ?? 0;
    },
    longDescription:'This is auto calculated start address. User may uncheck the auto option if manual start address needs to be put.'
}, {
    name: "manualStartAddress",
    displayName: "Start Address",
    hidden: true,
    default:  physicalLayout[Object.keys(physicalLayout)[0]].start,
    displayFormat: "hex",
 },
{
    name: "size",
    displayName: "Region Size (bytes)",
    default: 0x1,
    displayFormat: "hex",
    description: "Accepts hex format",
    description: 'In hexa-decimal'
},
{
    name: "endAddress",
    displayName: "End Address",
    default: 0x0,
    displayFormat: "hex",
    getValue: (inst) => {
        if(inst.auto) {
            return (inst.autoStartAddress + inst.size - 0x1 )
        }
        else {
            return (inst.manualStartAddress + inst.size - 0x1 )
        }
    }
},
{
    name: "alignment",
    displayName: "Alignment",
    description: "",
    longDescription: "",
    default: 8,
    hidden: true,
    displayFormat: "dec",
},
{
    name: "last_sym",
    displayName: "Last Symbol",
    default: "",
    longDescription:"Optionally specifies a Symbol that can be used at run-time to find the address of the last allocated byte in the memory \
    range"
},{
    name: "isShared",
    displayName: "Shared",
    default: false,
    hidden: !(g_memory_region_types[0].name.includes("_ALL") || ( physicalLayout[g_memory_region_types[0].name].access === "all")) ,
    onChange: enableShared_core,
    longDescription: 'Check if this region has to be shared among multiple cores. Once done, no need to select the same in other cores.'
 }, {
    name: "shared_cores",
    displayName: "Share With Cores",
    description: "",
    longDescription: "",
    default: [],
    hidden: true,
    options: (inst) => {return memoryRegs.coreList(inst)},
},
]

function valueCheck(inst, report){

    if(inst.size < 0) {
        report.logError("Size can't be less than 0", inst, "size")
    }

    let mem_params_size = physicalLayout[inst.type].size

    if(inst.size > mem_params_size) {
        report.logError(`Size can't be greater than ${mem_params_size} B`, inst, "size")
    }

    if(inst.size == 0) {
        report.logError(`Size can't be 0`, inst, "size")
    }

}


function checkSameFieldName(instance, report)
{
    let regions_this_core = instance.$module.$instances;
    let regions_shared_with_this_core = memoryRegs.util_function("")

    for (let i = 0; i < regions_this_core.length; i++) {
        if (instance.$name === regions_this_core[i].$name &&
            instance !== regions_this_core[i]) {
            report.logError(`Same name cannot be used`, instance, "$name");
        }
    }

    _.each(regions_shared_with_this_core, (remote_region) => {
            if(instance.$name === remote_region.$name) {
                report.logError(`Region with same name being shared by other core with this one`, instance, "$name");
            }
    })
}


function validate (inst, report) {
    if (-1 === getMemoryLayout()[system.context][inst.$name]) {
        report.logError("This region does not fit in the physical memory provided", inst, inst.auto ? "autoStartAddress" : "manualStartAddress")
    }

    checkSameFieldName(inst, report)
    valueCheck(inst, report)

    if (inst.isShared && inst.shared_cores.length == 0) {
        report.logError(`Select at least one core.`, inst, "shared_cores")
    }

    let selfCoreName = common.getSelfSysCfgCoreName();
    let coreNames = common.getSysCfgCoreNames();

    let other_core_list = coreNames.filter(function (coreName) {
        return coreName !== selfCoreName;
    });

    let invalid_option = false;

    if ( inst.shared_cores.length > 0 ){
        _.each(inst.shared_cores, shared_core => {
            if(other_core_list.indexOf(shared_core) == -1 )
            invalid_option = invalid_option || true;
        })
    }

    if (invalid_option)
    {
        report.logError(`Not a valid option.`, inst, "shared_cores")
    }
}

function func(inst) {
    return toString(inst.autoStartAddress)
}

let regionModule = {
    displayName: "Memory Region",
    defaultInstanceName: "MEMORY_REGION",
    config : config,
	validate: validate,
};

exports = regionModule