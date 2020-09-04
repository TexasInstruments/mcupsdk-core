
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

exports = {
    displayName: "TI Drivers",
    templates: [
        {
            name: "/drivers/system/system_config.c.xdt",
            outputPath: "ti_drivers_config.c",
            alwaysRun: true,
        },
        {
            name: "/drivers/system/system_config.h.xdt",
            outputPath: "ti_drivers_config.h",
            alwaysRun: true,
        },
        {
            name: "/drivers/system/drivers_open_close.c.xdt",
            outputPath: "ti_drivers_open_close.c",
            alwaysRun: true,
        },
        {
            name: "/drivers/system/drivers_open_close.h.xdt",
            outputPath: "ti_drivers_open_close.h",
            alwaysRun: true,
        },
        {
            name: "/drivers/pinmux/pinmux_config.c.xdt",
            outputPath: "ti_pinmux_config.c",
            alwaysRun: true,
        },
        {
            name: "/drivers/system/power_clock_config.c.xdt",
            outputPath: "ti_power_clock_config.c",
            alwaysRun: true,
        },
    ],
    topModules: soc.getTopModules(),
};
