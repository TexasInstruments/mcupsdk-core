

let common = system.getScript("/common");
let soc = system.getScript(`/board/eeprom/eeprom_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.name);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getConfigurables() {
    /* get 'CPU enable' configurables */
    let config = [];
    let staticConfig = soc.getConfigArr();

    config.push(
        {
            ...common.ui.makeConfig(staticConfig, "name", "EEPROM Device"),
            onChange: function(instance, ui) {

                let configArr = soc.getConfigArr();
                let config = configArr.find(o => o.name === instance.name);

                instance.i2cAddress = config.i2cAddress;
            },
        }
    );

    config.push(
        {
            name: "i2cAddress",
            displayName: "I2C Address",
            description: `I2C slave address of EEPROM`,
            default: staticConfig[0].i2cAddress,
            displayFormat: "hex",
        },
    );

    return config;
}

let eeprom_module_name = "/board/eeprom/eeprom";

let eeprom_module = {
    displayName: "EEPROM",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/eeprom/templates/eeprom_open_close_config.c.xdt",
            board_open: "/board/eeprom/templates/eeprom_open.c.xdt",
            board_close: "/board/eeprom/templates/eeprom_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/eeprom/templates/eeprom_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/eeprom/templates/eeprom.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_EEPROM",
    config: getConfigurables(),
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    sharedModuleInstances: moduleInstances,
    getInstanceConfig,
};

function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "i2cAddress", 0x0, 0x7F, "hex");
}

function moduleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "peripheralDriver",
        displayName: "I2C Configuration",
        moduleName: '/drivers/i2c/i2c',
    });

    return (modInstances);
}

exports = eeprom_module;
