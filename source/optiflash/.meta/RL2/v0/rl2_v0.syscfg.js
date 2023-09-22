let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/RL2/soc/rl2_${common.getSocName()}`);

/*
    This script export optiflash module.
    All settings that are required to be exported should be inside this struct.
*/
function validate(inst, report)
{
    let rangeStart =    parseInt(inst['rangeStart']);
    let rangeEnd =      parseInt(inst['rangeEnd']);
    let l2Sram0Base =   parseInt(inst['l2Sram0Base']);
    let l2Sram0Len =    parseInt(inst['l2Sram0Len']);
    let l2Sram1Base =   parseInt(inst['l2Sram1Base']);
    let l2Sram1Len =    parseInt(inst['l2Sram1Len']);
    let l2Sram2Base =   parseInt(inst['l2Sram2Base']);
    let l2Sram2Len =    parseInt(inst['l2Sram2Len']);
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

    resp = soc.rl2.validate_remote_address(l2Sram0Base);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram0Base");
    }

    resp = soc.rl2.validate_remote_size(l2Sram0Len);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram0Len");
    }

    resp = soc.rl2.validate_remote_address(l2Sram1Base);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram1Base");
    }

    resp = soc.rl2.validate_remote_size(l2Sram1Len);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram1Len");
    }

    resp = soc.rl2.validate_remote_address(l2Sram2Base);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram2Base");
    }

    resp = soc.rl2.validate_remote_size(l2Sram2Len);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "l2Sram2Len");
    }

    // check for intersection of different ranges.
    var ranges = [];
    if(l2Sram0Base !== 0 || l2Sram0Len !== 0)
    {
        ranges.push({ start: l2Sram0Base, end : l2Sram0Base + l2Sram0Len });
    }
    if(l2Sram1Base !== 0 || l2Sram1Len !== 0)
    {
        ranges.push({ start: l2Sram1Base, end : l2Sram1Base + l2Sram1Len });
    }
    if(l2Sram2Base !== 0 || l2Sram2Len !== 0)
    {
        ranges.push({ start: l2Sram2Base, end : l2Sram2Base + l2Sram2Len });
    }

    function checkIfRangesIntersect(ranges) {
        // Check if any of the ranges intersect.
        for (var i = 0; i < ranges.length; i++) {
            for (var j = 0; j < ranges.length; j++)
            {
                if (i !== j && (ranges[i].start <= ranges[j].end && ranges[i].end >= ranges[j].start))
                {
                    return true;
                }
            }
        }

        // No ranges intersect.
        return false;
    }

    if(checkIfRangesIntersect(ranges) === true)
    {
        report.logError("Ranges should not intersect each others.", inst, "l2Sram2Len");
    }

    // check if the total length amongst different ranges are equal to the size selected.
    let totalLen = l2Sram0Len + l2Sram1Len + l2Sram2Len;
    let reqLen = 0;
    switch(cacheLen)
    {
        case "RL2_CACHESIZE_8K":
            reqLen = 8*1024;
            break;

        case "RL2_CACHESIZE_16K":
            reqLen = 16*1024;
            break;

        case "RL2_CACHESIZE_32K":
            reqLen = 32*1024;
            break;

        case "RL2_CACHESIZE_64K":
            reqLen = 64*1024;
            break;

        case "RL2_CACHESIZE_128K":
            reqLen = 128 * 1024;
            break;

        case "RL2_CACHESIZE_256K":
            reqLen = 256 * 1024;
            break;
    }

    if(totalLen < reqLen)
    {
        report.logError("Size of cache should fit in memory allocated.", inst, "cacheSize");
    }
}


exports =
{
    displayName: "Layer2 Cache (RL2)",
    maxInstances: soc.rl2.max_instance(),
    defaultInstanceName: "RL2",
    config:[
        {
            name: "enable",
            displayName: "Enable",
            default: true
        },
        {
            name: "rangeStart",
            displayName: "Flash Cached Region Start Address",
            default: 0x88000000,
            displayFormat: "hex",
            description: "Starting address of external flash which is required to be cached."
        },
        {
            name: "rangeEnd",
            displayName: "Flash Cached Region End Address",
            default: 0x88100000,
            displayFormat: "hex",
            description: "End address of external flash which is required to be cached."
        },
        {
            name: "cacheSize",
            displayName: "Size Of Cache",
            default: "RL2_CACHESIZE_8K",
            options: [
                {name: "RL2_CACHESIZE_8K", displayName: "8K"},
                {name: "RL2_CACHESIZE_16K", displayName: "16K"},
                {name: "RL2_CACHESIZE_32K", displayName: "32K"},
                {name: "RL2_CACHESIZE_64K", displayName: "64K"},
                {name: "RL2_CACHESIZE_128K", displayName: "128K"},
                {name: "RL2_CACHESIZE_256K", displayName: "256K"},
            ],
            description: "Size of Cache."
        },
        {
            name: "l2Sram0Base",
            displayName: "Base Address Of Remote Region 0",
            default: 0,
            displayFormat: "hex",
        },
        {
            name: "l2Sram0Len",
            displayName: "Length Of Remote Region 0",
            default: 8*1024,
            displayFormat: "dec",
        },
        {
            name: "l2Sram1Base",
            displayName: "Base Address Of Remote Region 1",
            default: 0,
            displayFormat: "hex",
        },
        {
            name: "l2Sram1Len",
            displayName: "Length Of Remote Region 1",
            default: 0,
            displayFormat: "dec",
        },
        {
            name: "l2Sram2Base",
            displayName: "Base Address Of Remote Region 2",
            default: 0,
            displayFormat: "hex",
        },
        {
            name: "l2Sram2Len",
            displayName: "Length Of Remote Region 2",
            default: 0,
            displayFormat: "dec",
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
    }

};

