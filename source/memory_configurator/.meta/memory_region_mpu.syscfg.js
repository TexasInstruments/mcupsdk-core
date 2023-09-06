let common = system.getScript("/common");
let drivers = system.getScript("/drivers/drivers")
let general_module = system.modules['/memory_configurator/general'];
const { getMemoryLayout } = system.getScript("/memory_configurator/memoryLayoutSolver");
const physicalLayout = system.getScript("/memory_configurator/physicalLayout.json")[system.deviceData.device];

function calculateDefault(){

    let addrBase = physicalLayout.OCRAM.start
    let addrBaseFlash = physicalLayout.FLASH.start
    let codeDataAddr = 0
    let codeDataAddrFlash = 0
    let codeDataSize =  physicalLayout.OCRAM.size
    let codeDataSizeFlash = physicalLayout.FLASH.size
    let selfCoreName = common.getSelfSysCfgCoreName();


    if(selfCoreName == "r5fss0-0") {
        codeDataAddr    = addrBase + codeDataSize*0;
        codeDataAddrFlash = addrBaseFlash + codeDataSizeFlash*0;
    }
    if(selfCoreName == "r5fss0-1") {
        codeDataAddr    = addrBase + codeDataSize*1;
        codeDataAddrFlash = addrBaseFlash + codeDataSizeFlash*1;
    }
    if(selfCoreName == "r5fss1-0") {
        codeDataAddr    = addrBase + codeDataSize*2;
        codeDataAddrFlash = addrBaseFlash + codeDataSizeFlash*2;
    }
    if(selfCoreName == "r5fss1-1") {
        codeDataAddr    = addrBase + codeDataSize*3;
        codeDataAddrFlash = addrBaseFlash + codeDataSizeFlash*3;
    }


    let memory_params = {

        OCRAM: {
                start: codeDataAddr,
                size: codeDataSize
        },

        FLASH: {
                start: codeDataAddrFlash,
                size: codeDataSizeFlash
        },

        TCMA: {
            start: physicalLayout.TCMA.start,
            size:  physicalLayout.TCMA.size
        },

        TCMB: {
            start:  physicalLayout.TCMB.start,
            size: physicalLayout.TCMB.size
        },
    }

    return memory_params
}

function coreList() {
    let coreNames = common.getSysCfgCoreNames();
    coreNames = coreNames.filter(function (coreName) {
        return coreName.includes("r5f");
    });

    let selfCoreName = common.getSelfSysCfgCoreName();

    let tmp_list = coreNames.filter(function (coreName) {
        return coreName !== selfCoreName;
    });

    let options = [];

    for (let core of tmp_list)
    {
        let ele = {name: core, displayName: core}
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

function enableShared_core(inst, ui) {

    if(inst.isShared)
        ui.shared_cores.hidden = false;
    else
        ui.shared_cores.hidden = true;
}

function getDriversList()
{
    let topModules = drivers.topModules;
    const options = []
    _.each(topModules, (module) => {

        let result = module.substr(module.lastIndexOf('/')+1)
        options.push({name: result, displayName : result.toUpperCase()})
    })

    return options;
}

function update_opti_share(inst) {

    let shared_cores_list = inst.shared_cores
    if(shared_cores_list.length == 3) {
        inst.opti_share =  true
    }
    else {
        inst.opti_share =  false
    }
}

function update_shared_cores(inst)
{
    if(inst.opti_share) {
        let core_list = coreList();
        inst.shared_cores = (() => { let list = []
                                    _.each(core_list, (core) => { list.push(core.name) })
                                    return list;
                                })()
    }
    else {
        inst.shared_cores = []
    }
}

let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
    name: "type",
    displayName: "Type",
    default: "OCRAM",
    options: [{ name: "OCRAM" }, { name: "FLASH" }, { name: "TCMA" }, { name: "TCMB" }, { name: "CUSTOM" }],
    onChange: (inst) => {
        inst.$name = (inst.type.concat("_x")).toUpperCase()
    },
    longDescription: 'Choose CUSTOM if the memory address lies outside of the remaining types.'
}, {
    name: "auto",
    displayName: "Calculate Start Address",
    default: true,
    onChange: (inst) => {
        inst.$uiState.autoStartAddress.hidden = !inst.auto;
        inst.$uiState.manualStartAddress.hidden = inst.auto;
    },
    longDescription: 'Check this if placement of the region does not matter. The region will be placed in the smallest hole that fits it.'
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
    default: physicalLayout.OCRAM.start,
    displayFormat: "hex",
 },
 {
    name: "size",
    displayName: "Region Size (bytes)",
    default: 5,
    options: [
        {
            name: 5,
            displayName: "32 B"
        },
        {
            name: 6,
            displayName: "64 B"
        },
        {
            name: 7,
            displayName: "128 B"
        },
        {
            name: 8,
            displayName: "256 B"
        },
        {
            name: 9,
            displayName: "512 B"
        },
        {
            name: 10,
            displayName: "1 KB"
        },
        {
            name: 11,
            displayName: "2 KB"
        },
        {
            name: 12,
            displayName: "4 KB"
        },
        {
            name: 13,
            displayName: "8 KB"
        },
        {
            name: 14,
            displayName: "16 KB"
        },
        {
            name: 15,
            displayName: "32 KB"
        },
        {
            name: 16,
            displayName: "64 KB"
        },
        {
            name: 17,
            displayName: "128 KB"
        },
        {
            name: 18,
            displayName: "256 KB"
        },
        {
            name: 19,
            displayName: "512 KB"
        },
        {
            name: 20,
            displayName: "1 MB"
        },
        {
            name: 21,
            displayName: "2 MB"
        },
        {
            name: 22,
            displayName: "4 MB"
        },
        {
            name: 23,
            displayName: "8 MB"
        },
        {
            name: 24,
            displayName: "16 MB"
        },
        {
            name: 25,
            displayName: "32 MB"
        },
        {
            name: 26,
            displayName: "64 MB"
        },
        {
            name: 27,
            displayName: "128 MB"
        },
        {
            name: 28,
            displayName: "256 MB"
        },
        {
            name: 29,
            displayName: "512 MB"
        },
        {
            name: 30,
            displayName: "1 GB"
        },
        {
            name: 31,
            displayName: "2 GB"
        },
        {
            name: 32,
            displayName: "4 GB"
        },
    ],
},
{
    name: "endAddress",
    displayName: "End Address",
    default: 0x0,
    displayFormat: "hex",
    getValue: (inst) => {
        if(inst.auto) {
            return (inst.autoStartAddress + Math.pow(2,inst.size) - 0x1 )
        }
        else {
            return (inst.manualStartAddress + Math.pow(2,inst.size) - 0x1 )
        }
    }
},
{
    name: "alignment",
    displayName: "Alignment",
    description: "",
    longDescription: "",
    default: 8,
    displayFormat: "dec",
},{
    name: "isShared",
    displayName: "Shared",
    default: false,
    onChange: enableShared_core,
    longDescription: 'Check it if this region has to be shared among multiple cores. Once done, no need to select the same in other cores.'
 }, {
    name: "shared_cores",
    displayName: "Share with cores",
    description: "",
    longDescription: "",
    default: [],
    hidden: true,
    options: coreList,
    onChange: update_opti_share
},
// {
//     name: "drivers",
//     displayName: "Choose drivers",
//     description: "",
//     longDescription: "",
//     minSelections : 0,
//     default: [],
//     options: getDriversList()
// },
{
    name: "opti_share",
    displayName: "OptiShare",
    default: false,
    description:'',
    hidden: true,
    onChange: update_shared_cores
},
]

function valueCheck(inst, report){

    if(inst.size < 0) {
        report.logError("Size can't be less than 0", inst, "size")
    }

    let type = inst.type
    if(type != "CUSTOM") {
        let mem_params_size = calculateDefault()[type].size;

        if(inst.size > mem_params_size) {
            report.logError(`Size can't be greater than ${mem_params_size} B`, inst, "size")
        }
    }

    if(!inst.auto){
        if(inst.manualStartAddress % Math.pow(2,inst.size) != 0){
            report.logError(`Address has to be multiple of size`, inst, "manualStartAddress")
        }
    }

}

function util_function() {

    let coreNames =  common.getSysCfgCoreNames();
    let memory_region_module_name = '/memory_configurator/memory_region_mpu';
    let all_core_memory_instances = []
    let regions_shared_with_this_core = []
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
                regions_shared_with_this_core.push(each_instance)
            }
        })
    })

    return regions_shared_with_this_core
}

function checkSameFieldName(instance, report)
{
    let regions_this_core = instance.$module.$instances;
    let regions_shared_with_this_core = util_function()

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
}

function func(inst) {
    return toString(inst.autoStartAddress)
}

function addModuleInstances(inst) {
    let modInstances = new Array();
    let module_name = "kernel/dpl/mpu_armv7"

    modInstances.push({
        name: "mpu_config",
        displayName: "MPU ARMv7",
        moduleName: module_name,
        useArray: true,
        minInstanceCount: 0,
        maxInstanceCount: 1,
        collapsed: false,
        requiredArgs: {
            associated_mr: inst.$name,
            size: inst.size
        },
    });

    return modInstances;
}

let regionModule = {
    displayName: "Memory Region",
    defaultInstanceName: "OCRAM_x",
    config : config,
	validate: validate,
    moduleInstances: addModuleInstances,
};

exports = regionModule