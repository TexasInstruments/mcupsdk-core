
let common = system.getScript("/common");

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
            name: "size",
            displayName: "Region Size (bytes)",
            default: 32,
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
            ]
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
                    "displayName": "Cached+Sharable",
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

