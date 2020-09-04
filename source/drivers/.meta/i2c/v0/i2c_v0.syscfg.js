
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getStaticConfigArr() {
    return system.getScript(`/drivers/i2c/soc/i2c_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
};

function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "SCL", "I2C Clock Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "SDA", "I2C Data Pin"));

    let peripheral = {
        name          : interfaceName,
        displayName   : "I2C Instance",
        interfaceName : interfaceName,
        resources     : resources,
        signalTypes   : {
            sclPin     : ['SCL'],
            sdaPin     : ['SDA'],
        }
    };

    return [peripheral];
}

function getInterfaceName(inst) {

    if(inst.useMcuDomainPeripherals)
        return "MCU_I2C"

    return "I2C";
}

function getPeripheralPinNames(inst) {

    return [ "SCL", "SDA" ];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

let i2c_module_name = "/drivers/i2c/i2c";

let i2c_module = {
    displayName: "I2C",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/i2c/templates/i2c_v0_config.c.xdt",
            driver_init: "/drivers/i2c/templates/i2c_v0_init.c.xdt",
            driver_deinit: "/drivers/i2c/templates/i2c_v0_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/i2c/templates/i2c_v0.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/i2c/templates/i2c_v0_open_close_config.c.xdt",
            driver_open: "/drivers/i2c/templates/i2c_v0_open.c.xdt",
            driver_close: "/drivers/i2c/templates/i2c_v0_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/i2c/templates/i2c_v0_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: i2c_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: i2c_module_name,
        },
    },
    defaultInstanceName: "CONFIG_I2C",
    config: [
        {
            name: "bitRate",
            displayName: "Bit Rate",
            default: "400KHZ",
            description: "I2C bitrate",
            options: [
                {
                    name: "100KHZ",
                    displayName: "100 KHZ"
                },
                {
                    name: "400KHZ",
                    displayName: "400 KHZ"
                },
                {
                    name: "1P0MHZ",
                    displayName: "1 MHZ"
                },
                {
                    name: "3P4MHZ",
                    displayName: "3.4 MHZ"
                },
            ],
        },
        {
            name: "enableIntr",
            displayName: "Enable Interrupt",
            default: true,
            onChange: function (inst, ui) {
                let hideConfigs = false;
                if(inst.enableIntr == false) {
                    hideConfigs = true;
                }
                ui.transferMode.hidden = hideConfigs;
            },
            description: "If enabled interrupt mode otherwise polling mode",
        },
        {
            name: "transferMode",
            displayName: "Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.transferMode == "CALLBACK") {
                    hideConfigs = false;
                }
                ui.transferCallbackFxn.hidden = hideConfigs;
            },
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "transferCallbackFxn",
            displayName: "Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Transfer callback function when callback mode is selected",
        },
        common.getUseMcuDomainPeripheralsConfig(),
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.ownSlaveAddr1.hidden = hideConfigs;
                ui.ownSlaveAddr2.hidden = hideConfigs;
                ui.ownSlaveAddr3.hidden = hideConfigs;
                ui.ownSlaveAddr4.hidden = hideConfigs;
            },
        },
        {
            name: "ownSlaveAddr1",
            displayName: "Own Slave Address 1 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownSlaveAddr2",
            displayName: "Own Slave Address 2 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownSlaveAddr3",
            displayName: "Own Slave Address 3 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownSlaveAddr4",
            displayName: "Own Slave Address 4 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
    ],
    validate : validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
};

function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "ownSlaveAddr1", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownSlaveAddr2", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownSlaveAddr3", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownSlaveAddr4", 0x0, 0x7F, "hex");
    common.validate.checkValidCName(instance, report, "transferCallbackFxn");
    if((instance.transferMode == "CALLBACK") &&
        ((instance.transferCallbackFxn == "NULL") ||
            (instance.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", instance, "transferCallbackFxn");
    }
}

exports = i2c_module;
