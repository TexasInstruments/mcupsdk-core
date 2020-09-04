
let common = system.getScript("/common");
let soc = system.getScript(`/kernel/dpl/addr_translate_${common.getSocName()}`);

let addr_translate_module = {
    displayName: "RAT",
    longDescription: soc.getLongDescription(),
    maxInstances: 16,
    templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
            dpl_config: "/kernel/dpl/addr_translate.c.xdt",
            dpl_init: "/kernel/dpl/addr_translate_init.c.xdt",
        },
        "/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/addr_translate.h.xdt",
    },
    defaultInstanceName: "CONFIG_ADDR_TRANSLATE_REGION",
    config: [
        {
            name: "localAddr",
            displayName: "Local Address (hex)",
            description: "MUST be <= 32 bits and MUST be region size aligned and MUST NOT overlap with other regions",
            default: 0x0,
            displayFormat: "hex",
        },
        {
            name: "systemAddr",
            displayName: "Translated System Address (hex)",
            description: "MUST be <= 48 bits and MUST be region size aligned",
            default: 0x0,
            displayFormat: "hex",
        },
        {
            name: "size",
            displayName: "Region Size (bytes)",
            default: 29,
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
    ],

    validate : function (instance, report) {
        let localAddr = instance.localAddr;
        let systemAddr = instance.systemAddr;
        let size = 2**instance.size;
        let maxLocalAddr = 0x100000000;
        let maxSystemAddr = 0x1000000000000;

        if ( (localAddr % size) != 0)
        {
            report.logError( `Region start address must be aligned to 0x${size.toString(16).toUpperCase()}`,
                    instance, "localAddr");
        }
        if ( (systemAddr % size) != 0)
        {
            report.logError( `Translated Region start address must be aligned to 0x${size.toString(16).toUpperCase()}`,
                    instance, "systemAddr");
        }
        if (localAddr >= maxLocalAddr)
        {
            report.logError( `Region start address must be <= 0x${(maxLocalAddr-1).toString(16).toUpperCase()}`,
                    instance, "localAddr");
        }
        if (systemAddr >= maxSystemAddr)
        {
            report.logError( `Translated Region start address must be <= 0x${(maxSystemAddr-1).toString(16).toUpperCase()}`,
                    instance, "systemAddr");
        }
        /* check overlap across instances */
        module = instance.$module;

        for( let curInstance of module.$instances)
        {
            if(curInstance!=instance)
            {
                let curSize = 2**curInstance.size;
                let size = 2**instance.size;
                if( (    instance.localAddr >= curInstance.localAddr
                      && instance.localAddr < (curInstance.localAddr+curSize)
                    ) 
                    ||  
                    (    curInstance.localAddr >= instance.localAddr
                        && curInstance.localAddr < (instance.localAddr+size)
                      )                    
                    )
                {
                    report.logError( `Overlapping regions not allowed`,
                        instance, "localAddr");
                }
            }
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
    getRatBaseAddr() {
        return soc.getRatBaseAddr();
    },
};

exports = addr_translate_module;

