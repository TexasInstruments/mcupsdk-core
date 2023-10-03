let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/RAT/soc/rat_${common.getSocName()}`);

function validate(inst, report)
{
    let ba = parseInt(inst['baseAddress']);
    let ta = parseInt(inst['translatedAddress']);
    let size = inst['size'];
    let size_in_bytes = 0;
    switch(size)
    {
        case "AddrTranslateP_RegionSize_4K":
            size_in_bytes = 4*1024;
            break;
        case "AddrTranslateP_RegionSize_8K":
            size_in_bytes = 8*1024;
            break;
        case "AddrTranslateP_RegionSize_16K":
            size_in_bytes = 16*1024;
            break;
        case "AddrTranslateP_RegionSize_32K":
            size_in_bytes = 32*1024;
            break;
        case "AddrTranslateP_RegionSize_64K":
            size_in_bytes = 64*1024;
            break;
        case "AddrTranslateP_RegionSize_128K":
            size_in_bytes = 128*1024;
            break;
        case "AddrTranslateP_RegionSize_256K":
            size_in_bytes = 256*1024;
            break;
        case "AddrTranslateP_RegionSize_512K":
            size_in_bytes = 512*1024;
            break;
        case "AddrTranslateP_RegionSize_1M":
            size_in_bytes = 1024*1024;
            break;
        case "AddrTranslateP_RegionSize_2M":
            size_in_bytes = 2*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_4M":
            size_in_bytes = 4*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_8M":
            size_in_bytes = 8*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_32M":
            size_in_bytes = 32*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_64M":
            size_in_bytes = 64*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_128M":
            size_in_bytes = 128*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_256M":
            size_in_bytes = 256*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_512M":
            size_in_bytes = 512*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_1G":
            size_in_bytes = 1024*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_2G":
            size_in_bytes = 2*1024*1024*1024;
            break;
        case "AddrTranslateP_RegionSize_4G":
            size_in_bytes = 4*1024*1024*1024;
            break;
    }
    size_in_bytes -= 1;
    if((ba & size_in_bytes) != 0)
    {
        report.logError("Base Address should be aligned to size", inst, "baseAddress");
    }
    if((ta & size_in_bytes) != 0)
    {
        report.logError("Translated Address should be aligned to size", inst, "translatedAddress");
    }
}

exports =
{
    displayName: "Region Address Translation (RAT)",
    maxInstances: soc.rat.maxInstances,
    defaultInstanceName: "RAT",
    config:
    [
        {
            name: "size",
            displayName: "Region Size",
            options: [
                {name: "AddrTranslateP_RegionSize_4K", displayName: "4K"},
                {name: "AddrTranslateP_RegionSize_8K", displayName: "8K"},
                {name: "AddrTranslateP_RegionSize_16K", displayName: "16K"},
                {name: "AddrTranslateP_RegionSize_32K", displayName: "32K"},
                {name: "AddrTranslateP_RegionSize_64K", displayName: "64K"},
                {name: "AddrTranslateP_RegionSize_128K", displayName: "128K"},
                {name: "AddrTranslateP_RegionSize_256K", displayName: "256K"},
                {name: "AddrTranslateP_RegionSize_512K", displayName: "512K"},
                {name: "AddrTranslateP_RegionSize_1M", displayName: "1M"},
                {name: "AddrTranslateP_RegionSize_2M", displayName: "2M"},
                {name: "AddrTranslateP_RegionSize_4M", displayName: "4M"},
                {name: "AddrTranslateP_RegionSize_8M", displayName: "8M"},
                {name: "AddrTranslateP_RegionSize_16M", displayName: "16M"},
                {name: "AddrTranslateP_RegionSize_32M", displayName: "32M"},
                {name: "AddrTranslateP_RegionSize_64M", displayName: "64M"},
                {name: "AddrTranslateP_RegionSize_128M", displayName: "128M"},
                {name: "AddrTranslateP_RegionSize_256M", displayName: "256M"},
                {name: "AddrTranslateP_RegionSize_512M", displayName: "512M"},
                {name: "AddrTranslateP_RegionSize_1G", displayName: "1G"},
                {name: "AddrTranslateP_RegionSize_2G", displayName: "2G"},
                {name: "AddrTranslateP_RegionSize_4G", displayName: "4G"}
            ],
            default: "AddrTranslateP_RegionSize_4K",
            description: "Size of the region that is to be translated"
        },
        {
            name: "baseAddress",
            displayName: "Region Base Address (hex)",
            default: 0,
            displayFormat: "hex"
        },
        {
            name: "translatedAddress",
            displayName: "Region Translated Address (hex)",
            default: 0,
            displayFormat: "hex"
        }
    ],
    validate: validate,
    templates:
    {
        "/drivers/system/system_config.c.xdt":
        {

            driver_config: "/optiflash/RAT/templates/rat_config.c.xdt",
            driver_init: "/optiflash/RAT/templates/rat_init.c.xdt",
            driver_deinit: "/optiflash/RAT/templates/rat_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt":
        {
            driver_config: "/optiflash/RAT/templates/rat.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt":
        {
            driver_open_close_config: "/optiflash/RAT/templates/rat_open_close_config.c.xdt",
            driver_open: "/optiflash/RAT/templates/rat_open.c.xdt",
            driver_close: "/optiflash/RAT/templates/rat_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt":
        {
            driver_open_close_config: "/optiflash/RAT/templates/rat_open_close.h.xdt",
        }
    }
};