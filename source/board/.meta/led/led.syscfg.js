
let common = system.getScript("/common");
let soc = system.getScript(`/board/led/led_${common.getSocName()}`);
let gpio = system.getScript("/drivers/gpio/gpio");

function getInstanceConfig(moduleInstance) {
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.name);

    let additionalConfig =  {
        gpioBaseAddr: "CSL_MCU_GPIO0_BASE",//`CSL_${getInstanceString(moduleInstance)}_BASE`,
        gpioPinNum: 5,//gpio.getPinIndex(moduleInstance.peripheralDriver.$interface),
    }

    return {
        ...config,
        ...moduleInstance,
        ...additionalConfig,
    };
};

function getConfigurables() {
    /* get 'CPU enable' configurables */
    let config = [];
    let staticConfig = soc.getConfigArr();

    config.push(
        {
            ...common.ui.makeConfig(staticConfig, "name", "LED"),
            onChange: function(instance, ui) {
                let configArr = soc.getConfigArr();
                let config = configArr.find(o => o.name === instance.name);

                instance.i2cAddress = config.i2cAddress;
                let hideConfigs = true;
                if((instance.name == "TPIC2810") || (instance.name == "Ioexp")) {
                    hideConfigs = false;
                }
                ui.i2cAddress.hidden = hideConfigs;

                hideConfigs = true;
                if(instance.name == "Ioexp") {
                    hideConfigs = false;
                }
                ui.ioIndex.hidden = hideConfigs;
            },
        }
    );
    config.push(
        {
            name: "i2cAddress",
            displayName: "I2C Address",
            description: `I2C slave address of LED controller`,
            default: staticConfig[0].i2cAddress,
            hidden: true,
            displayFormat: "hex",
        },
    );
    config.push(
        {
            name: "ioIndex",
            displayName: "IO Expander Pin Index",
            description: `IO Expander Pin Index to which the LED is connected`,
            default: staticConfig[0].ioIndex,
            hidden: true,
        },
    );

    return config;
}

let led_module_name = "/board/led/led";

let led_module = {
    displayName: "LED",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/led/templates/led_open_close_config.c.xdt",
            board_open: "/board/led/templates/led_open.c.xdt",
            board_close: "/board/led/templates/led_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/led/templates/led_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/led/templates/led.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_LED",
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
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === instance.name);

    if(config.type == "GPIO") {
        modInstances.push({
            name: "peripheralDriver",
            displayName: "GPIO Configuration",
            moduleName: '/drivers/gpio/gpio',
        });
    }

    if(config.type == "I2C") {
        modInstances.push({
            name: "peripheralDriver",
            displayName: "I2C Configuration",
            moduleName: '/drivers/i2c/i2c',
        });
    }

    return (modInstances);
}

exports = led_module;
