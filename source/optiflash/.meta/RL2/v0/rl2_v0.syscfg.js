let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/RL2/soc/rl2_${common.getSocName()}`);

const linkerStartSymbol =  "__RL2_" + toCVariableName(common.getSelfSysCfgCoreName()) + "_cachebank_start";
const linkerEndSymbol = "__RL2_" + toCVariableName(common.getSelfSysCfgCoreName()) + "_cachebank_end";

function toCVariableName(str) {
    let result = '';
    for (let i = 0; i < str.length; i++) {
        let char = str[i];
        if (/^[a-zA-Z0-9_]$/.test(char)) {
            result += char;
        }
    }
    return result;
}

function convertRL2CacheSizeTokenToNumbercacheLen(cacheLen) {
    let reqLen = 0;
    switch (cacheLen) {
        case "RL2_CACHESIZE_8K":
            reqLen = 8 * 1024;
            break;

        case "RL2_CACHESIZE_16K":
            reqLen = 16 * 1024;
            break;

        case "RL2_CACHESIZE_32K":
            reqLen = 32 * 1024;
            break;

        case "RL2_CACHESIZE_64K":
            reqLen = 64 * 1024;
            break;

        case "RL2_CACHESIZE_128K":
            reqLen = 128 * 1024;
            break;

        case "RL2_CACHESIZE_256K":
            reqLen = 256 * 1024;
            break;
    }

    return reqLen;
}

/*
    This script export optiflash module.
    All settings that are required to be exported should be inside this struct.
*/
function validate(inst, report) {

    let rangeStart =    parseInt(inst['rangeStart']);
    let rangeEnd =      parseInt(inst['rangeEnd']);
    let cacheLen =      inst["cacheSize"];

    let resp;

    resp = soc.rl2.validate_src_range_start_address(rangeStart);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst,"rangeStart");
    }

    resp = soc.rl2.validate_src_range_start_address(rangeEnd);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "rangeEnd");
    }
    else if(rangeEnd <= rangeStart)
    {
        report.logError("Should not be less than or equal to external flash cached range end address.", inst, "rangeEnd");
    }

    let valid_option = false;
    let valid_mr_options = loadMemoryRegions().memory_regions;

    _.each(valid_mr_options, vmr =>{
        if( inst.memoryRegion.length > 0 && vmr.name === inst.memoryRegion )
            valid_option = valid_option || true;
    })

    if (!valid_option)
    {
        report.logError(`Not a valid option.`, inst, "memoryRegion")
    }
}

function loadMemoryRegions() {

    /*  List should include all the MRs defined in this core as well as shared by other cores with this core*/
    let memory_regions = []

    let coreNames = common.getSysCfgCoreNames();
    let memory_region_module_name = ""
    let region_module = system.modules['/memory_configurator/region'];
    if (region_module !== undefined) {
        if (region_module.$instances[0].mpu_setting)
            memory_region_module_name = "/memory_configurator/memory_region_mpu";
        else
            memory_region_module_name = "/memory_configurator/memory_region";

        let selfCoreName = common.getSelfSysCfgCoreName();
        for (let core of coreNames) {

            let core_module = common.getModuleForCore(memory_region_module_name, core);
            let core_module_instances;
            if (core_module != undefined) {
                core_module_instances = core_module.$instances;
                _.each(core_module_instances, instance => {

                    let obj = {
                        name: " ",
                        opti_share: false
                    }
                    let displayName = instance.$name.concat(" Type: ", instance.type)
                    //displayName = displayName.concat("Opti-share ", instance.opti_share)
                    if (core.includes(selfCoreName)) {
                        memory_regions.push({ name: instance.$name, displayName: displayName })
                        obj.name = instance.$name
                        obj.opti_share = instance.opti_share
                    }
                    else if (instance.isShared && instance.shared_cores.includes(selfCoreName)) {
                        memory_regions.push({ name: instance.$name, displayName: displayName })
                        obj.name = instance.$name
                        obj.opti_share = instance.opti_share
                    }
                })
            }
        }
    }
    else {
        memory_regions.push({ name: "", displayName: "" })
    }
    return { memory_regions }
}

function dummyFunc(inst) {
    let cacheSize = convertRL2CacheSizeTokenToNumbercacheLen(inst.cacheSize);
    let linkerStr = ".rl2CacheBank: { . = . + " + cacheSize + ";} palign(4096)";

    return linkerStr;
}

function addModuleInstances(inst) {
    let modInstances = new Array();
    let module_name = "memory_configurator/section"

    modInstances.push({
        name: "memory_section",
        displayName: "Memory Section",
        moduleName: module_name,
        useArray: false,
        collapsed: true,
        requiredArgs: {
            load_memory: inst.memoryRegion,
            run_memory: inst.memoryRegion,
            $name: "RL2CacheBank",
            type: "NOLOAD",
            group: true,
            group_start: linkerStartSymbol,
            group_end: linkerEndSymbol,
            load_to_memory: "Memory",
            select_multiple_regions: false,
            split_across_memories: false,
        },
        args: {
            generic_text: dummyFunc(inst)
        }
    });

    return modInstances;
}

exports =
{
    displayName: "Layer2 Cache (RL2)",
    maxInstances: soc.rl2.max_instance(),
    defaultInstanceName: "RL2",
    config: [
        {
            name: "enable",
            displayName: "Enable",
            default: true
        },
        {
            name: "rangeStart",
            displayName: "Flash Cached Region Start Address",
            default: 0x60000000,
            displayFormat: "hex",
            description: "Starting address of external flash which is required to be cached."
        },
        {
            name: "rangeEnd",
            displayName: "Flash Cached Region End Address",
            default: 0x68000000,
            displayFormat: "hex",
            description: "End address of external flash which is required to be cached."
        },
        {
            name: "cacheSize",
            displayName: "Size Of Cache",
            default: "RL2_CACHESIZE_8K",
            options: [
                { name: "RL2_CACHESIZE_8K", displayName: "8K" },
                { name: "RL2_CACHESIZE_16K", displayName: "16K" },
                { name: "RL2_CACHESIZE_32K", displayName: "32K" },
                { name: "RL2_CACHESIZE_64K", displayName: "64K" },
                { name: "RL2_CACHESIZE_128K", displayName: "128K" },
                { name: "RL2_CACHESIZE_256K", displayName: "256K" },
            ],
            description: "Size of Cache."
        },
        {
            name: "memoryRegion",
            displayName: "Memory Region",
            default: "",
            description: 'Chose in which Memory cache bank for this L2 cache will locate. Mostly L2 memory',
            options: () => { return loadMemoryRegions().memory_regions },
        },
    ],
    validate: validate,
    templates:
    {
        "/drivers/system/system_config.c.xdt":
        {
            driver_config: "/optiflash/RL2/templates/rl2_config.c.xdt",
            driver_init: "/optiflash/RL2/templates/rl2_init.c.xdt",
            driver_deinit: "/optiflash/RL2/templates/rl2_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt":
        {
            driver_config: "/optiflash/RL2/templates/rl2.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt":
        {
            driver_open_close_config: "/optiflash/RL2/templates/rl2_open_close_config.c.xdt",
            driver_open: "/optiflash/RL2/templates/rl2_open.c.xdt",
            driver_close: "/optiflash/RL2/templates/rl2_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt":
        {
            driver_open_close_config: "/optiflash/RL2/templates/rl2_open_close.h.xdt",
        }
    },
    moduleInstances: addModuleInstances,
    toCVariableName: toCVariableName,
    rl2_mem_start:linkerStartSymbol,
    rl2_mem_end: linkerEndSymbol
};

