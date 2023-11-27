
let common = system.getScript("/common");
const memoryRegs = system.getScript("/memory_configurator/helper");
const module = system.modules['/kernel/dpl/mpu_armv7'];
const physicalLayout = system.getScript("/memory_configurator/physicalLayout.json")[system.deviceData.device];

function memoryRegionMapping(mp_instance) {
    let selfCoreName = common.getSelfSysCfgCoreName();

    let mr_list = {
        name: "",
        type: "",
        shared: false,
        cores: "",
        access_check: true
    }

    let all_core_mr_instances = util_function()
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

    let mp_start = mp_instance.baseAddr
    let mp_size = Math.pow(2, mp_instance.size)
    let mp_end = Number(mp_start) + Number(mp_size)

    _.each(sorted_memory_regions, (region) => {

        let mr_start = ( region.auto ? region.autoStartAddress: region.manualStartAddress )
        let mr_end =  mr_start + region.size - 0x1


        if (mr_end < mp_start || mp_end < mr_start)  // current mp instance doesn't fall inside this mr
        { }
        else
        {
            let region_name = (region.$name).concat("_",region.core)
            mr_list.name = (mr_list.name).concat(region_name, "\n")
            if(mr_list.type.includes(region.type) == false)
                mr_list.type = (mr_list.type).concat(region.type, "\n")

            if(mr_list.cores.includes(region.core) == false)
                mr_list.cores = (mr_list.cores).concat(region.core, "\n")

            if(region.isShared)
            {
                mr_list.shared = true
                let arr_cores = region.shared_cores

                if(!region.core.includes(selfCoreName) && !arr_cores.includes(selfCoreName)) {
                    mr_list.access_check = false;
                }

                _.each( arr_cores, (core) => {
                    if(mr_list.cores.includes(core) == false)
                    mr_list.cores = (mr_list.cores).concat(core, " ")
                } )
            }
        }
    })

    return mr_list;
}

function displayName(inst) {

    let mr_names = memoryRegionMapping(inst).name;
    let mr_names_array = mr_names.split("\n");
    let map = {};

    _.each(mr_names_array, (name) => {

        let lastIndex = name.lastIndexOf("_");
        let region_name = name.substring(0, lastIndex);
        let core_name = name.substring(lastIndex+1);

        if(map.hasOwnProperty(core_name)) {
            map[core_name].push(region_name);
        }
        else {
            map[core_name] = [region_name];
        }
    })

    let ans = "";

    for(let key in map) {
        if (map.hasOwnProperty(key)) {
            if(key.length > 0) {
                ans = ans.concat(key.toUpperCase()," --> \n\n")
                let value = map[key];
                _.each(value, ele => {
                    ans = ans.concat(ele, "\n")
            })
            ans = ans.concat("\n");
            }
        }
    }

    return ans;
}

function util_function() {

    let coreNames =  common.getSysCfgCoreNames();
    let memory_region_module_name = '/memory_configurator/memory_region'

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

function readFromMemoryRegion(inst) {

    let baseAddr = 0x0
    if(inst.$ownedBy !== undefined) {

        baseAddr = inst.$ownedBy.manualStartAddress;
        if(inst.$ownedBy.auto){
            baseAddr = inst.$ownedBy.autoStartAddress;
        }
    }
    return baseAddr;
}

function changeDisplay(inst) {

    if(inst.$ownedBy !== undefined) {
        inst.$uiState.baseAddr_mr.hidden = false;
        inst.$uiState.associated_mr.hidden = false;

        inst.$uiState.baseAddr.hidden = true;
        inst.$uiState.memoryRegion_name.hidden = true;
        inst.$uiState.memoryRegion_type.hidden = true;
        inst.$uiState.memoryRegion_shared.hidden = true;
        inst.$uiState.memoryRegion_core.hidden = true;
    }
    else {
        inst.$uiState.baseAddr_mr.hidden = true;
        inst.$uiState.associated_mr.hidden = true;

        inst.$uiState.baseAddr.hidden = false;
        inst.$uiState.memoryRegion_name.hidden = false;
        inst.$uiState.memoryRegion_type.hidden = false;
        inst.$uiState.memoryRegion_shared.hidden = false;
        inst.$uiState.memoryRegion_core.hidden = false;
    }
}

function checkConflictingConfig(report, instance){

    let selfCoreName = common.getSelfSysCfgCoreName();
    const module = system.modules['/kernel/dpl/mpu_armv7'];
    let module_instances = module.$instances;
    let shareable_cacheable_conflict = false;
    let access_permission_conflict = false;
    let excute_permission_conflict = false;
    let region_config_violation = false;
    let mpu_set = ""

    if ( instance.$ownedBy ){
        mpu_set = "_mpu"
    }
    _.each(module_instances, mpu_instance => {

        let mr_list = memoryRegs.memoryRegionInformation(mpu_instance, mpu_set)
        let mpu_accessPermissions = mpu_instance.accessPermissions
        let mpu_attributes = mpu_instance.attributes

        for(let i = 0; i < mr_list.name.length; i++ ){
            if( mpu_attributes == "Cached" || mpu_attributes == "Cached+Sharable"){
                if(mr_list.shared[i] == true){
                    shareable_cacheable_conflict = true;
                }
            }

            if( mpu_accessPermissions.includes("RD+WR") && !mr_list.permissions[i].includes("W")){
                access_permission_conflict = true;
            }

            if( mpu_instance.allowExecute && !mr_list.permissions[i].includes("X")){
                excute_permission_conflict = true;
            }

            if( mr_list.cores[i] != selfCoreName && mr_list.shared[i] == false){
                if(physicalLayout[mr_list.type[i]].access == "all"){ // Had "access" been "individual", it would mean it's specific to this core
                    region_config_violation = true;
                }
            }
        }
    })

    if( shareable_cacheable_conflict ) {
        report.logInfo( `Some memory region(s) within this range is Shared among cores. `, instance, "attributes");
    }

    if( access_permission_conflict ) {
        report.logInfo( `Some memory region(s) within this range might not have the correct access permissions. `, instance, "accessPermissions");
    }

    if( excute_permission_conflict ) {
        report.logInfo( `Some memory region(s) within this range might not have execute permissions. `, instance, "allowExecute");
    }

    if( region_config_violation ) {
        report.logInfo( `This MPU region includes other cores' memory regions as well. Pl. make sure what you configure.`, instance, "$name");
    }
}

let mpu_armv7_module = {
    displayName: "MPU ARMv7",
    longDescription: "Refer to ARMv7-R or AMRv-M Architecture Technical Reference Manual for more details",
    maxInstances: 16,
    templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
            dpl_config: "/kernel/dpl/mpu_armv7.c.xdt",
            dpl_init: "/kernel/dpl/mpu_armv7_init.c.xdt",
        },
        "/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/mpu_armv7.h.xdt",
    },
    defaultInstanceName: "CONFIG_MPU_REGION",
    config: [
        {
            name: "baseAddr",
            displayName: "Region Start Address (hex)",
            description: "MUST be <= 32 bits and MUST be region size aligned",
            default: 0x0,
            displayFormat: "hex",
        },
        {
            name: "baseAddr_mr",
            displayName: "Region Start Address (hex)",
            description: "MUST be <= 32 bits and MUST be region size aligned",
            default: 0x0,
            hidden: true,
            displayFormat: "hex",
            getValue: readFromMemoryRegion
        },
        {
            name: "associated_mr",
            displayName: "Memory Region",
            description: "",
            hidden: true,
            default: "",
        },
        {
            name: "size",
            displayName: "Region Size (bytes)",
            default: 32,
            options: [
                {
                    name: 0,
                    displayName: "0 B"
                },
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
            onChange: changeDisplay
        },
        {
            name: "accessPermissions",
            displayName: "Access Permissions",
            default: "Supervisor RD+WR, User RD+WR",
            options: [
                {
                    "name": "Supervisor RD+WR, User RD+WR",
                },
                {
                    "name": "Supervisor RD+WR, User RD",
                },
                {
                    "name": "Supervisor RD, User RD",
                },
                {
                    "name": "Supervisor RD+WR, User BLOCK",
                },
                {
                    "name": "Supervisor RD, User BLOCK",
                },
                {
                    "name": "Supervisor BLOCK, User BLOCK",
                },
            ],
        },
        {
            name: "attributes",
            displayName: "Region Attributes",
            default: "Cached",
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.attributes == "CUSTOM") {
                    hideConfigs = false;
                }
                ui.tex.hidden = hideConfigs;
                ui.isCacheable.hidden = hideConfigs;
                ui.isBufferable.hidden = hideConfigs;
                ui.isShareable.hidden = hideConfigs;
            },
            options: [
                {
                    "name": "Device",
                    "displayName": "Strongly Ordered",
                    "description": "Cached disabled. Strongly orderd. Use for peripheral MMRs and non-cached shared regions.",
                },
                {
                    "name": "NonCached",
                    "displayName": "Non Cached",
                    "description": "Cache disabled. Outer and inner non-cacheable. Use for non-cached shared regions.",
                },
                {
                    "name": "Cached",
                    "displayName": "Cached",
                    "description": "Cache enabled. Outer and inner write-back, write-allocate. Use for cached code/data regions.",
                },
                {
                    "name": "Cached+Sharable",
                    "displayName": "Shareable",
                    "description": "Cache enabled and region marked as sharable. Outer and inner write-back, write-allocate.",
                },
                {
                    "name": "CUSTOM",
                    "displayName": "Advanced Configuration",
                    "description": "Precisely set TEX[2:0], C, B, S region attribute bits.",
                },
            ]
        },
        {
            name: "allowExecute",
            displayName: "Allow Code Execution",
            default: true,
            description: "Keep this set for code sections. Keep this un-set for data only sections or peripheral MMR sections"
        },
        {
            name: "subregionDisableMask",
            displayName: "Sub-Region Disable Mark (hex)",
            default: 0x0,
            displayFormat: "hex",
            description: "Value MUST be between 0x00 .. 0xFF"
        },
        {
            name: "tex",
            displayName: "Type Extention TEX[2:0]",
            default: 1,
            hidden: true,
            description: "Refer ARMv7-R/M architecture manual for more details",
            options: [
                {
                    name: 0,
                    displayName: "000",
                },
                {
                    name: 1,
                    displayName: "001",
                },
                {
                    name: 2,
                    displayName: "010",
                },
                {
                    name: 3,
                    displayName: "011",
                },
                {
                    name: 4,
                    displayName: "100",
                },
                {
                    name: 5,
                    displayName: "101",
                },
                {
                    name: 6,
                    displayName: "110",
                },
                {
                    name: 7,
                    displayName: "111",
                },
            ]
        },
        {
            name: "isCacheable",
            displayName: "Cacheable (C)",
            default: true,
            hidden: true,
            description: "Refer ARMv7-R/M architecture manual for more details",
        },
        {
            name: "isBufferable",
            displayName: "Bufferable (B)",
            default: true,
            hidden: true,
            description: "Refer ARMv7-R/M architecture manual for more details",
        },
        {
            name: "isShareable",
            displayName: "Shareable (S)",
            default: false,
            hidden: true,
            description: "Refer ARMv7-R/M architecture manual for more details",
        },
        {
            name: "group_memory_region",
            displayName: "Memory Region",
            description: " ",
            collapsed: false,
            config : [
                {
                    name: "memoryRegion_name",
                    displayName: "Name",
                    multiline: true,
                    default: "",
                    description: 'Displays all the regions core wise which fall within this range.',
                    readOnly: true,
                    getValue: (inst) => { return displayName(inst); }
                },
                {
                    name: "memoryRegion_type",
                    displayName: "Type",
                    multiline: true,
                    default: "",
                    description: 'Displays types of all the regions core wise which fall wthin this range.',
                    readOnly: true,
                    getValue: (inst) => { return memoryRegionMapping(inst).type ; }
                },
                {
                    name: "memoryRegion_shared",
                    displayName: "Shared",
                    default: false,
                    description: 'If checked, it means at least one of the regions falling in this range is shared.',
                    readOnly: true,
                    getValue: (inst) => { return memoryRegionMapping(inst).shared ; }
                },
                {
                    name: "memoryRegion_core",
                    displayName: "Cores Using It",
                    multiline: true,
                    default: " ",
                    description: 'Displays all the cores that uses the regions falling within this range.',
                    readOnly: true,
                    getValue: (inst) => { return memoryRegionMapping(inst).cores ; }
                },
            ]
        }
    ],

    validate : function (instance, report) {
        let startAddr = instance.baseAddr;
        let size = 2**instance.size;
        let maxAddr = 0x100000000;

        common.validate.checkNumberRange(instance, report, "subregionDisableMask", 0, 0xFF, "hex");

        if ( (startAddr % size) != 0)
        {
            report.logError( `Region start address must be aligned to 0x${size.toString(16).toUpperCase()}`,
                    instance, "baseAddr");
        }
        if (startAddr >= maxAddr)
        {
            report.logError( `Region start address must be <= 0x${(maxAddr-1).toString(16).toUpperCase()}`,
                    instance, "baseAddr");
        }

        // mpu_access_violation(instance, report);
        checkConflictingConfig(report, instance)
    },
    getSizeString(size) {
        size = 2**size;
        if(size < 1024) {
            return size.toString();
        }
        else
        if(size < 1024*1024) {
            size = size/1024;
            return size.toString()+"K";
        }
        else
        if(size < 1024*1024*1024) {
            size = size/(1024*1024);
            return size.toString()+"M";
        }
        else {
            size = size/(1024*1024*1024);
            return size.toString()+"G";
        }
    },
	moduleStatic: {
		modules: function(inst) {
			return [{
				name: "system_common",
				moduleName: "/system_common",
			}]
		},
    },
    getRegionAttributes(instance) {

        let attributes = {};

        if(instance.attributes == "Cached+Sharable")
        {
            attributes.isCacheable = 1;
            attributes.isBufferable = 1;
            attributes.isSharable = 1;
            attributes.tex = 1;
        }
        else
        if(instance.attributes == "Cached")
        {
            attributes.isCacheable = 1;
            attributes.isBufferable = 1;
            attributes.isSharable = 0;
            attributes.tex = 1;
        }
        else
        if(instance.attributes == "NonCached")
        {
            attributes.isCacheable = 0;
            attributes.isBufferable = 0;
            attributes.isSharable = 1;
            attributes.tex = 1;
        }
        else
        if(instance.attributes == "CUSTOM")
        {
            attributes.isCacheable = Number(instance.isCacheable);
            attributes.isBufferable = Number(instance.isBufferable);
            attributes.isSharable = Number(instance.isShareable);
            attributes.tex = Number(instance.tex);
        }
        else
        {
            attributes.isCacheable = 0;
            attributes.isBufferable = 0;
            attributes.isSharable = 1;
            attributes.tex = 0;
        }
        switch(instance.accessPermissions) {
            default:
            case "Supervisor RD+WR, User RD+WR":
                attributes.accessPerm = "MpuP_AP_ALL_RW";
                break;
            case "Supervisor RD+WR, User RD":
                attributes.accessPerm = "MpuP_AP_S_RW_U_R";
                break;
            case "Supervisor RD, User RD":
                attributes.accessPerm = "MpuP_AP_ALL_R";
                break;
            case "Supervisor RD+WR, User BLOCK":
                attributes.accessPerm = "MpuP_AP_S_RW";
                break;
            case "Supervisor RD, User BLOCK":
                attributes.accessPerm = "MpuP_AP_S_R";
                break;
            case "Supervisor BLOCK, User BLOCK":
                attributes.accessPerm = "MpuP_AP_ALL_BLOCK";
                break;
            }
        attributes.isExecuteNever = !instance.allowExecute;
        attributes.subregionDisableMask = instance.subregionDisableMask;
        return attributes;
    },
    isCacheAvailable() {
        let cpu = common.getSelfSysCfgCoreName();

        if(cpu.match(/m4f*/))
            return false;

        return true;
    },
};

exports = mpu_armv7_module;
