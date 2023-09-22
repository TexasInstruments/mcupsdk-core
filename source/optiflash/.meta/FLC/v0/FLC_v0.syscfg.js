let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/FLC/soc/FLC_${common.getSocName()}`);

function validate(inst, report)
{
    let ssa = parseInt(inst['sourceStartAddress']);
    let dsa = parseInt(inst['destinationStartAddress']);
    let size = parseInt(inst['copySize']);

    let resp ;

    resp = soc.flc.validate_source_address(ssa);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst,"sourceStartAddress")
    }

    resp = soc.flc.validate_source_address(dsa);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst,"destinationStartAddress")
    }

    resp = soc.flc.validate_copy_size(size);
    if(resp.status === false)
    {
        report.logError(resp.msg, inst, "copySize");
    }

    if(size === 0)
    {
        report.logError("Size should not be 0", inst,"copySize")
    }
};

exports =
{
    displayName: "Fast Local Copy",
    maxInstances: soc.flc.max_instance(),
    defaultInstanceName: "FLC",
    config:[
        {
            name: "enable",
            displayName: "Enabled",
            default: true
        },
        {
            name: "sourceStartAddress",
            displayName: "Source Start Address",
            default: 0,
            displayFormat: "hex",
            description: "For the range that is required to be copied, enter starting address of that range. Ususlly this is flash address."
        },
        {
            name: "destinationStartAddress",
            displayName: "Destination Starting Address",
            default: 0,
            displayFormat: "hex",
            description: "This is the destination address where it is be copied."
        },
        {
            name: "copySize",
            displayName: "Size (Bytes)",
            default: 4096,
            displayFormat: "dec",
            description: "This is the size that is requried to be copied. This value should be 4096 aligned."
        }
    ],
    validate: validate,
    templates:
    {
        "/drivers/system/system_config.c.xdt":
        {
            driver_config: "/optiflash/FLC/templates/flc_config.c.xdt",
            driver_init: "/optiflash/FLC/templates/flc_init.c.xdt",
            driver_deinit: "/optiflash/FLC/templates/flc_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt":
        {
            driver_config: "/optiflash/FLC/templates/flc.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt":
        {
            driver_open_close_config: "/optiflash/FLC/templates/flc_open_close_config.c.xdt",
            driver_open: "/optiflash/FLC/templates/flc_open.c.xdt",
            driver_close: "/optiflash/FLC/templates/flc_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt":
        {
            driver_open_close_config: "/optiflash/FLC/templates/flc_open_close.h.xdt",
        }
    }
};

