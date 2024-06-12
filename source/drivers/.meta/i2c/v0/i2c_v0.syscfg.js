
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

    if(inst.useWakeUpDomainPeripherals)
        return "WKUP_I2C"

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
                    inst.transferCallbackFxn = "NULL";
                    inst.transferMode = "BLOCKING";
                    ui.transferCallbackFxn.hidden = true;
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
                if(inst.transferMode == "CALLBACK") {
                    ui.transferCallbackFxn.hidden = false;
                }
                else{
                    inst.transferCallbackFxn = "NULL";
                    ui.transferCallbackFxn.hidden = true;
                }
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
        common.getUseWakeupDomainPeripheralsConfig(),
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.ownTargetAddr1.hidden = hideConfigs;
                ui.ownTargetAddr2.hidden = hideConfigs;
                ui.ownTargetAddr3.hidden = hideConfigs;
                ui.ownTargetAddr4.hidden = hideConfigs;
            },
        },
        {
            name: "ownTargetAddr1",
            displayName: "Own Target Address 1 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownTargetAddr2",
            displayName: "Own Target Address 2 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownTargetAddr3",
            displayName: "Own Target Address 3 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "ownTargetAddr4",
            displayName: "Own Target Address 4 (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "sdkInfra",
            displayName: "SDK Infra",
            default: "HLD",
            options: [
                {
                    name: "HLD",
                    displayName: "HLD"
                },
                {
                    name: "LLD",
                    displayName: "LLD"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.sdkInfra == "LLD") {
                    inst.transferMode = "BLOCKING";
                    inst.transferCallbackFxn = "NULL";
                    ui.transferCallbackFxn.hidden = true;
                    ui.transferMode.hidden = true;
                    inst.enableIntr = false;
                    ui.enableIntr.hidden = true;
                    if(inst.enableIntr == "NULL") {
                        /* Clear NULL entry as user need to provide a fxn */
                        inst.enableIntr = false;
                    }
                }
                else {
                    ui.enableIntr.hidden = false;
                    inst.enableIntr = false;
                }
            },
            description: "SDK Infra",
            hidden: (common.getSocName() == "am65x"),
        },
    ],
    validate : validate,
    moduleInstances: moduleInstances,
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
    common.validate.checkNumberRange(instance, report, "ownTargetAddr1", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownTargetAddr2", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownTargetAddr3", 0x0, 0x7F, "hex");
    common.validate.checkNumberRange(instance, report, "ownTargetAddr4", 0x0, 0x7F, "hex");
    common.validate.checkValidCName(instance, report, "transferCallbackFxn");
    if((instance.transferMode == "CALLBACK") &&
        ((instance.transferCallbackFxn == "NULL") ||
            (instance.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", instance, "transferCallbackFxn");
    }
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    if( inst.sdkInfra == "HLD")
    {
        modInstances.push({
            name: "I2C_child",
            moduleName: '/drivers/i2c/v0/i2c_v0_template',
            },
        );
    }
    else
    {
        modInstances.push({
            name: "I2C_child",
            moduleName: '/drivers/i2c/v0/i2c_v0_template_lld',
            },
        );
    }

    return (modInstances);
}

exports = i2c_module;
