let common = system.getScript("/common");
let soc = system.getScript(`/board/psram/soc/psram_${common.getSocName()}`);

function getDriver(drvName) {
    return system.getScript(`/drivers/${drvName}/${drvName}`);
}

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let psram_module = {
    displayName: "PSRAM",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/psram/templates/psram_open_close_config.c.xdt",
            board_open: "/board/psram/templates/psram_open.c.xdt",
            board_close: "/board/psram/templates/psram_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/psram/templates/psram_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/psram/templates/psram.h.xdt",
        },

    },
    maxInstances: 1,
    defaultInstanceName: "CONFIG_PSRAM",
    config: [
        {
            name: "pname",
            displayName: "PSRAM Name",
            default: soc.getDefaultPsramName(),
            placeholder: "Type your psram name here",
        },
        {
            name: "deviceWidth",
            displayName: "Device Size",
            default: "16 bit",
            options: [
                { name: "16 bit" },
            ],
        },
        /* Psram Config */
        {
            name: "basicPsramCfg",
            displayName: "Basic Psram Configuration",
            collapsed: true,
            config: [
                {
                    name: "psramSize",
                    displayName: "PSRAM Size In Bytes",
                    default: soc.getDefaultPsramConfig().psramSize,
                    displayFormat: "dec",
                },
            ]
        }
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    validate: validate,
    moduleInstances: moduleInstances,
    getInstanceConfig,
};

function validate(inst, report) {

    common.validate.checkSameFieldName(inst, "device", report);

}

function moduleInstances(inst) {

    let modInstances = new Array();
    let requiredArgs = {
        deviceWidth: inst.deviceWidth,
    };

    modInstances.push({
        name: "psramDriverIoExpander",
        displayName: "PSRAM Configuration",
        moduleName: "/drivers/i2c/i2c",
        useArray: false,
    })
    modInstances.push({
            name: "psramDriver",
            displayName: "PSRAM Configuration",
            moduleName: "/drivers/gpmc/gpmc",
            useArray: false,
        })

    return (modInstances);
}

exports = psram_module;