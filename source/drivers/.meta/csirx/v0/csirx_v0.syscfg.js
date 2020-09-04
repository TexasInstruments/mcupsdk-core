
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/csirx/soc/csirx_${common.getSocName()}`);

function getInstanceHwAttrs(inst)
{
    return soc.getInstanceHwAttrs(inst);
}

function enableContextInterrupt(context)
{
    let enableInterrupt = false;

    if(   context.isNumLines
       || context.isFramesToAcquire
       || context.isPayloadChecksumMismatch
       || context.isLineStartCodeDetect
       || context.isLineEndCodeDetect
       || context.isFrameStartCodeDetect
       || context.isFrameEndCodeDetect
       || context.isLongPacketOneBitErrorCorrect
        )
    {
       enableInterrupt = true;
    }
    return  enableInterrupt;
}

let csirx_module = {
    displayName: "CSIRX",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/csirx/templates/csirx_config.c.xdt",
            driver_init: "/drivers/csirx/templates/csirx_init.c.xdt",
            driver_deinit: "/drivers/csirx/templates/csirx_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/csirx/templates/csirx.h.xdt",
        },
         "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/csirx/templates/csirx_open_close_config.c.xdt",
            driver_open: "/drivers/csirx/templates/csirx_open.c.xdt",
            driver_close: "/drivers/csirx/templates/csirx_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/csirx/templates/csirx_open_close.h.xdt",
        },
    },

    defaultInstanceName: "CONFIG_CSIRX",
    maxInstances: soc.getStaticConfigArr().length,
    config: [
        common.ui.makeConfig( soc.getStaticConfigArr(), "instance", "HW Instance"),
        {
            name: "phyEnable",
            displayName: "Enable Phy Initialization",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "instOpenEnable",
            displayName: "Enable Instance Open in Drivers_open",
            default: true,
            skipTests: ["displayNameCheck"],
            longDescription:
`When checked, calling 'Drivers_open' will open and initialize the CSIRX driver.
 When unchecked, users need to call 'Drivers_csirxInstanceOpen' in their application explicitly for this CSIRX instance.`
        }
    ],
    validate: validate,
    moduleInstances: moduleInstances,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceHwAttrs,
    enableContextInterrupt,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

function moduleInstances(inst) {

    let modInstances = new Array();

    modInstances.push({
        name: "dphy",
        displayName: "CSIRX DPHY Configuration",
        moduleName: '/drivers/csirx/v0/csirx_v0_dphy',
        useArray: false,
        collapsed: false,
    });

    modInstances.push({
        name: "complexio",
        displayName: "CSIRX Complex IO Configuration",
        moduleName: '/drivers/csirx/v0/csirx_v0_complexio',
        useArray: false,
        collapsed: false,
    });

    modInstances.push({
        name: "common",
        displayName: "CSIRX Common Configuration",
        moduleName: '/drivers/csirx/v0/csirx_v0_common',
        useArray: false,
        collapsed: false,
    });

    modInstances.push({
        name: "context",
        displayName: "CSIRX Context Configuration",
        moduleName: '/drivers/csirx/v0/csirx_v0_context',
        useArray: true,
        maxInstanceCount: 8,
        minInstanceCount: 1,
        defaultInstanceCount: 1,
        collapsed: false,
    });

    return (modInstances);

}

exports = csirx_module;
