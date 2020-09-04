
let common = system.getScript("/common");

function getRegionAttributes(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let cache_c6x_mar_module = {
    displayName: "Memory Attribute Register (MAR) Configuration",
    longDescription: `
This adds and configures a block of region with MAR attributes.
When regions overlap, the entry that is occurs later will take effect.
So in case of overlap create regions in decending order of size.
    `,
    alwaysShowLongDescription: true,
    config: [
        {
            name: "baseAddr",
            displayName: "Region Start Address (hex)",
            default: 0x10000000,
            description: "MUST be <= 32 bits, MUST be 16 MB aligned and MUST be >= 240 MB",
            displayFormat: "hex",
        },
        {
            name: "size",
            displayName: "Region Size (MBytes)",
            default: 16,
            description: "MUST be multiple of 16 MB",
        },
        {
            name: "enableCache",
            displayName: "Enable Cache",
            default: false,
            description: "Sets the PC bit to make the region cacheable"
        },
        {
            name: "enablePrefetch",
            displayName: "Enable Prefetch",
            default: false,
            description: "Sets the PFX bit to make the region prefetchable"
        },
    ],
    validate: validate,
    getRegionAttributes,
};

/*
 *  ======== validate ========
 */
function validate(instance, report) {
    let startAddr = instance.baseAddr;
    let size_bytes = 1024 * 1024 * instance.size;
    let maxAddr = 0x100000000;
    let mar_size = 16 * 1024 * 1024;        // 16MB size
    let minAddr = 12 * mar_size;            // First 12 MAR regions are reserved
    let minAddrCache = 16 * mar_size;       // First 16 MAR regions cannot enable cache

    if(startAddr < minAddr) {
        report.logError(`Region start address must be >= 0x${minAddr.toString(16).toUpperCase()}`,
                instance, "baseAddr");
    }
    if(startAddr >= maxAddr) {
        report.logError(`Region start address must be <= 0x${(maxAddr-1).toString(16).toUpperCase()}`,
                instance, "baseAddr");
    }
    if((startAddr+size_bytes) > maxAddr) {
        report.logError(`Region end address must be <= 0x${(maxAddr-1).toString(16).toUpperCase()}`,
                instance, "size");
    }
    if(startAddr % mar_size) {
        report.logError(`Region start address must be aligned to 0x${mar_size.toString(16).toUpperCase()}`,
                instance, "baseAddr");
    }
    if((startAddr < minAddrCache) && (instance.enableCache)) {
        report.logError(`Region start address must be >= 0x${minAddrCache.toString(16).toUpperCase()} to enable Cache`,
                instance, "baseAddr");
    }
    if(size_bytes % mar_size) {
        report.logError(`Size must be multiple of 16 MB`, instance, "size");
    }
}

exports = cache_c6x_mar_module;
