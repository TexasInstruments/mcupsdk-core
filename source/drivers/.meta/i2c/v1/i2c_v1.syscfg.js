
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/i2c/soc/i2c_${common.getSocName()}`);

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

    return soc.getInterfaceName(inst);
}

function getPeripheralPinNames(inst) {

    return [ "SCL", "SDA" ];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getConfigurables()
{
    let config = [];

    config.push(

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
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.ownSlaveAddr.hidden = hideConfigs;
            },
        },
        {
            name: "ownSlaveAddr",
            displayName: "Own Slave Address (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
    )

    if(soc.isMakeInstanceRequired())
    {
        config.push(common.ui.makeInstanceConfig(getStaticConfigArr()));
    }

    return config;

}

let i2c_module_name = "/drivers/i2c/i2c";

let i2c_module = {
    displayName: "I2C",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/i2c/templates/i2c_v1_config.c.xdt",
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
    config:  getConfigurables(),
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
    validatePinmux: validatePinmux,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
};

function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "ownSlaveAddr", 0x0, 0x7F, "hex");
    common.validate.checkValidCName(instance, report, "transferCallbackFxn");
    if((instance.transferMode == "CALLBACK") &&
        ((instance.transferCallbackFxn == "NULL") ||
            (instance.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", instance, "transferCallbackFxn");
    }
}

/*
 *  ======== validatePinmux ========
 */
function validatePinmux(instance, report) {
    let instConfig = getInstanceConfig(instance);
    if((instance.enableIntr == true) &&
        (instConfig.name == "MSS_I2CA") &&
            (common.getSelfSysCfgCoreName().includes("c66"))){
        report.logError("Interrupt mode is not supported for this instance", instance, "enableIntr");
    }
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function getModule()
{
    let module = i2c_module;
    if(soc.isFrequencyDefined())
    {
        module.getClockFrequencies = getClockFrequencies;
    }
    return module;
}

exports = getModule();
